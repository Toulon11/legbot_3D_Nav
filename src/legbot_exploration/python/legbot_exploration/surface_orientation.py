"""Transient normals of real returns; no surface fitting/filling is stored."""
import numpy as np
from scipy.spatial import cKDTree


def classify(points, origin, radius=.25, minimum=6, vertical=.5, thickness=.03):
    """+1 upward support, -1 underside, -2 vertical, 0 insufficient evidence.

    Orient each local normal towards the timestamp-matched sensor origin.
    Covariances are temporary frame-local measurements, never map evidence.
    """
    points=np.asarray(points,dtype=float)
    result=np.zeros(len(points),dtype=np.int8)
    if len(points)<minimum:return result
    tree=cKDTree(points)
    for start in range(0,len(points),2048):
        batch=points[start:start+2048]
        distances,ids=tree.query(batch,k=min(12,len(points)),distance_upper_bound=radius,workers=1)
        valid=ids<len(points);ids=np.minimum(ids,len(points)-1)
        count=valid.sum(axis=1)
        samples=points[ids];mean=(samples*valid[:,:,None]).sum(1)/np.maximum(count[:,None],1)
        delta=(samples-mean[:,None,:])*valid[:,:,None]
        covariance=np.einsum('nki,nkj->nij',delta,delta)/np.maximum(count[:,None,None],1)
        values,vectors=np.linalg.eigh(covariance)
        normals=vectors[:,:,0]
        facing=np.einsum('ni,ni->n',normals,np.asarray(origin)-mean)
        normals=np.where((facing<0)[:,None],-normals,normals)
        usable=(count>=minimum)&(values[:,1]>1e-6)&(values[:,0]<=thickness*thickness)&\
               (values[:,0]<=.15*values[:,1])&(abs(facing)>.01)
        labels=np.where(normals[:,2]>=vertical,1,np.where(normals[:,2]<=-vertical,-1,-2))
        # At grazing incidence a small fitted tilt can reverse the sign.
        # Require the sensor to have viewed a horizontal face from its side.
        height_delta=origin[2]-batch[:,2]
        usable &= (labels==-2) | (labels*height_delta>.03)
        result[start:start+len(batch)]=np.where(usable,labels,0)
    return result
