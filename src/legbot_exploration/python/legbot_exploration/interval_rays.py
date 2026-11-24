"""Free voxel intervals from actual sensor-to-return segments, never range alone."""
import math


def trace_free_columns(points, origin, resolution, origin_x, origin_y, origin_z,
                       max_range, max_voxels):
    """Small serial reference for the native 3-D voxel traversal.

    XY voxels use the map's centre-based indexing. Z voxels are half-open
    intervals of length resolution. The return's voxel is never cleared.
    Rays that meet an edge/corner enter only the next voxel, not side voxels
    touched at a single point. This is intentionally conservative.
    """
    offset=(origin_x-resolution*.5,origin_y-resolution*.5,origin_z)
    result={};count=0
    start=tuple(float(v) for v in origin)
    for point in points:
        delta=tuple(float(b)-a for a,b in zip(start,point))
        length=math.sqrt(sum(v*v for v in delta))
        if length<=1e-9:continue
        reach=min(length,max_range)
        direction=tuple(v/length for v in delta)
        cell=[math.floor((v-o)/resolution) for v,o in zip(start,offset)]
        hit=tuple(math.floor((float(v)-o)/resolution) for v,o in zip(point,offset))
        step=[1 if d>0 else -1 if d<0 else 0 for d in direction]
        dt=[resolution/abs(d) if d else math.inf for d in direction]
        crossing=[((offset[i]+(cell[i]+(step[i]>0))*resolution-start[i])/direction[i]
                   if direction[i] else math.inf) for i in range(3)]
        distance=0.
        while distance<reach-1e-9 and tuple(cell)!=hit:
            following=min(crossing)
            if min(following,reach)-distance>1e-9:
                bucket=result.setdefault(tuple(cell[:2]),set())
                if cell[2] not in bucket:
                    bucket.add(cell[2]);count+=1
                    if count>max_voxels:raise ValueError('pct_free_voxel_limit')
            for i in range(3):
                if crossing[i]<=following+1e-10:
                    cell[i]+=step[i];crossing[i]+=dt[i]
            distance=following
    return {xy:compress(sorted(zs)) for xy,zs in result.items()}


def compress(values):
    spans=[]
    for z in values:
        if spans and z<=spans[-1][1]+1:spans[-1]=(spans[-1][0],max(z,spans[-1][1]))
        else:spans.append((z,z))
    return spans


def merge_intervals(first, second):
    spans=[]
    for low,high in sorted((*first,*second)):
        if spans and low<=spans[-1][1]+1:spans[-1]=(spans[-1][0],max(high,spans[-1][1]))
        else:spans.append((low,high))
    return tuple(spans)
