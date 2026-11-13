"""Measured G/C equivalence to original GPU kernels on the same fixed lattice.

Costs intentionally differ: finite gradients and explicit unknown inflation
replace original sentinel-gradient behaviour. No equality of those costs claimed.
"""
from pathlib import Path
from copy import deepcopy
import sys
import numpy as np
from legbot_exploration.online_pct import OnlinePct,_load


def main():
    project=Path(__file__).resolve().parents[3]
    root=project/'install/pct_planner/share/pct_planner'
    backend=OnlinePct(str(root))  # Preloads project-local CUDA libraries.
    backend.configure(dict(maturity_min_points=1,surface_noise_tolerance=0.))  # Compare original extrema with noise handling disabled.
    try:
        import cupy as cp
        if cp.cuda.runtime.getDeviceCount()<1: return 77
    except Exception as error:
        print('SKIP original GPU G/C comparison:',error);return 77
    _load('_test_gc_config',root/'tomography/config/__init__.py',True)
    cfg=deepcopy(_load('_test_gc_config.scene_go2',root/'tomography/config/scene_go2.py').SceneGo2)
    sys.path.insert(0,str(root/'tomography/scripts'))
    original=_load('_test_gc_mapper',root/'tomography/scripts/tomogram.py').Tomogram
    # Use cell centres and non-boundary Z values so this comparison tests
    # measured extrema independently from CPU/GPU half-cell rounding ties.
    rng=np.random.default_rng(21)
    frames=[np.column_stack((rng.integers(-b,b,(2000,2))*.1,rng.uniform(-.3,z,2000))).astype('float32')
            for b,z in ((10,1.5),(30,2.5),(35,3.5))]
    for version,frame in enumerate(frames,1):
        backend.integrate(memoryview(frame));backend.prepare(b'',version)
        m=backend.map
        exported=m.export_dense();dims=list(exported['G'].shape[1:])
        center=np.asarray([m.settings.origin_x,m.settings.origin_y])+(m.lower+np.asarray(dims)//2)*m.settings.resolution
        mapper=original(cfg)
        mapper.initMappingEnv(center,*dims,m.last_layer-m.first_layer+1,m.settings.origin_z+m.first_layer*m.settings.slice_dh)
        mapper.point2map(np.vstack(frames[:version]))
        g=mapper.layers_g.get();c=mapper.layers_c.get()
        np.testing.assert_array_equal(np.isfinite(exported['G']),g>-1e6)
        np.testing.assert_allclose(exported['G'][np.isfinite(exported['G'])],g[g>-1e6],atol=1e-6)
        retained=np.zeros(g.shape,dtype=bool)
        for (x,y),layers in m.column_layers.items():
            for layer in layers:
                retained[layer-m.first_layer,x-m.lower[0],y-m.lower[1]]=True
        # Sparse columns omit distant lower cuts containing only C and no G.
        # Every actual G and every locally retained G/C representation must
        # still equal the original kernels; omitted cuts cannot be walkable.
        assert not (g[~retained]>-1e6).any()
        np.testing.assert_array_equal(np.isfinite(exported['C'][retained]),c[retained]<1e6)
        valid=retained & np.isfinite(exported['C'])
        np.testing.assert_allclose(exported['C'][valid],c[valid],atol=1e-6)
        print('PASS fixed-grid G/C vs original GPU, version',version,'surface records',m.surface_records)
    return 0


if __name__=='__main__':sys.exit(main())
