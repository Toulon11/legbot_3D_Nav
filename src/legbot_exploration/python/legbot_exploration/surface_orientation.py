"""Observed surface sides from local geometry and the synchronized viewpoint."""
import numpy as np
from scipy.spatial import cKDTree


def classify(points, origin):
    """+1 upper face, -1 underside, -2 steep face, 0 insufficient evidence.

    Orient a locally fitted normal toward the observing LiDAR. Absolute point
    height relative to that LiDAR is never a ground/ceiling classifier. Keep
    sparse, mixed and grazing returns without promoting them to open ground.
    """
    points=np.asarray(points,dtype=float).reshape(-1,3)
    origin=np.asarray(origin,dtype=float)
    if origin.shape!=(3,) or not np.isfinite(origin).all():
        raise ValueError('pct_invalid_sensor_origin')
    result=np.zeros(len(points),dtype=np.int8)
    if len(points)<6:return result
    # One representative per 2.5 cm voxel bounds duplicate-hit work and avoids
    # treating six identical returns as a measured plane.
    _,selected,inverse=np.unique(np.floor(points/.025).astype(np.int64),axis=0,
                                 return_index=True,return_inverse=True)
    samples=points[selected]
    if len(samples)<6:return result
    tree=cKDTree(samples)
    distances,indices=tree.query(samples,k=min(16,len(samples)),distance_upper_bound=.25)
    valid=np.isfinite(distances)
    neighbors=samples[np.minimum(indices,len(samples)-1)]
    count=valid.sum(axis=1)
    mean=(neighbors*valid[:,:,None]).sum(axis=1)/np.maximum(count,1)[:,None]
    delta=(neighbors-mean[:,None,:])*valid[:,:,None]
    covariance=np.einsum('nki,nkj->nij',delta,delta)/np.maximum(count,1)[:,None,None]
    values,vectors=np.linalg.eigh(covariance)
    normals=vectors[:,:,0]
    planar=(count>=6)&(values[:,1]>1e-6)&(values[:,0]<=.03*np.maximum(values.sum(axis=1),1e-12))
    sight=origin-mean
    side=np.einsum('ni,ni->n',normals,sight)
    # A grazing ray cannot establish which face of a horizontal sheet was seen.
    # At long, grazing range a small normal error can invert the observed
    # side. Reject statistically ambiguous signs instead of using absolute Z
    # as a fallback. This also prevents false low ceilings on noisy ground.
    angular_error=np.sqrt(np.maximum(values[:,0],0.)/np.maximum(count*values[:,1],1e-12))
    uncertainty=3.*np.linalg.norm(sight,axis=1)*angular_error
    observed=planar&(np.abs(side)>np.maximum(.005,uncertainty))
    normals*=np.where(side>=0.,1.,-1.)[:,None]
    roles=np.zeros(len(samples),dtype=np.int8)
    roles[observed & (normals[:,2]>.5)]=1
    roles[observed & (normals[:,2]<-.5)]=-1
    roles[observed & (np.abs(normals[:,2])<=.5)]=-2
    return roles[inverse]
