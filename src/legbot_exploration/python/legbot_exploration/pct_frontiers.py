"""Frontiers derived solely from native PCT ground/ceiling/cost channels.

A frontier is an observed, potentially traversable surface bordering a missing
height-compatible surface. It is an observation boundary, not proof of free
space beyond it. Native inflated cost selects inland observation positions;
only native A* decides whether those positions are connected to the robot.
"""
from collections import deque
import numpy as np


def extract_block_frontiers(mapping, observation_radius=2.):
    """Full unknown-side rebuild for offline comparison, not production."""
    from .pct_frontier_index import rebuild_frontiers
    index=rebuild_frontiers(mapping,observation_radius)
    return index.query()


def extract_frontiers(planner, step_max, interval_min, observation_radius=2.0, barrier_query=None):
    cost, _, _, ground, ceiling = planner.tomogram
    resolution = planner.resolution
    valid = np.isfinite(ground) & (ground > -99)
    potential = valid & (~np.isfinite(ceiling) | (ceiling-ground >= interval_min))
    safe = potential & np.isfinite(cost) & (cost <= 20) & (cost >= 0)
    layers, nx, ny = ground.shape
    boundary = np.zeros_like(potential)
    # Compare actual elevations across all retained slices. Slice numbers do
    # not identify floors: several slices can represent the very same floor.
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
        xs, ys = slice(max(0,-dx),min(nx,nx-dx)), slice(max(0,-dy),min(ny,ny-dy))
        xt, yt = slice(max(0,dx),min(nx,nx+dx)), slice(max(0,dy),min(ny,ny+dy))
        for layer in range(layers):
            h = ground[layer,xs,ys]
            compatible = np.zeros(h.shape,dtype=bool)
            blocked = np.zeros(h.shape,dtype=bool)
            for other in range(layers):
                g, c = ground[other,xt,yt], ceiling[other,xt,yt]
                compatible |= potential[other,xt,yt] & (np.abs(g-h) <= step_max+1e-5)
                # Observed geometry in the body interval is a known barrier,
                # not an unexplored surface. An overhead floor alone is not.
                blocked |= (np.isfinite(g) & (g > h+step_max+1e-5) & (g <= h+interval_min))
                blocked |= (np.isfinite(c) & (c >= h-step_max) & (c < h+interval_min))
            pending=potential[layer,xs,ys] & ~compatible & ~blocked
            if barrier_query is not None and pending.any():
                i,j=np.nonzero(pending)
                # PCT may discard intermediate slices used to recognize a
                # wall beside an upper floor. Query only these sparse columns
                # in the original PCT G/C, not an additional occupancy map.
                barrier=barrier_query(i+xs.start+dx,j+ys.start+dy,h[pending])
                pending[i[barrier],j[barrier]]=False
            boundary[layer,xs,ys] |= pending
    # Map edges count as unknown, rather than accidentally reporting complete.
    boundary[:,0,:] |= potential[:,0,:]; boundary[:,-1,:] |= potential[:,-1,:]
    boundary[:,:,0] |= potential[:,:,0]; boundary[:,:,-1] |= potential[:,:,-1]
    columns = {}
    nodes = []
    for layer,i,j in np.argwhere(boundary):
        height = float(ground[layer,i,j])
        bucket = columns.setdefault((int(i),int(j)),[])
        if any(abs(nodes[n][3]-height) <= 0.02 for n in bucket):
            continue
        bucket.append(len(nodes)); nodes.append((int(layer),int(i),int(j),height))
    clusters, seen = [], set()
    for first in range(len(nodes)):
        if first in seen:
            continue
        seen.add(first); queue=deque([first]); cluster=[]
        while queue:
            n=queue.popleft(); cluster.append(nodes[n])
            _,i,j,h=nodes[n]
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    if dx == 0 and dy == 0:
                        continue
                    for other in columns.get((i+dx,j+dy),()):
                        if other not in seen and abs(nodes[other][3]-h) <= step_max+1e-5:
                            seen.add(other); queue.append(other)
        clusters.append(cluster)
    output=[]
    for number,cluster in enumerate(clusters):
        cells=np.asarray(cluster,dtype=float)
        xy=(cells[:,1:3]-planner.offset)*resolution+planner.center
        xyz=np.column_stack((xy,cells[:,3]))
        representative=int(np.argmin(np.linalg.norm(xyz-np.median(xyz,axis=0),axis=1)))
        # Sampling affects observer ranking and visualization only. All frontier
        # cells participate in clustering and unresolved/completion counts.
        seeds=xyz[np.linspace(0,len(xyz)-1,min(len(xyz),64),dtype=int)]
        lo=np.maximum(0,cells[:,1:3].min(0).astype(int)-int(np.ceil(observation_radius/resolution)))
        hi=np.minimum([nx,ny],cells[:,1:3].max(0).astype(int)+int(np.ceil(observation_radius/resolution))+1)
        choices={}
        indices=np.argwhere(safe[:,lo[0]:hi[0],lo[1]:hi[1]])
        indices[:,1:]+=lo
        # Bounded vector batches avoid a Python loop over every dense map cell
        # and bound temporary distances to 512 x 64 per cluster.
        for offset in range(0,len(indices),512):
            batch=indices[offset:offset+512]
            pos=(batch[:,1:]-planner.offset)*resolution+planner.center
            heights=ground[batch[:,0],batch[:,1],batch[:,2]]
            distances=np.linalg.norm(pos[:,None,:]-seeds[None,:,:2],axis=2)
            distances[np.abs(heights[:,None]-seeds[None,:,2]) > step_max+1e-5]=np.inf
            nearest=distances.min(axis=1)
            for n in np.flatnonzero(nearest <= observation_radius):
                layer,i,j=map(int,batch[n]);h=float(heights[n])
                rank=abs(float(nearest[n])-0.8)+0.01*float(cost[layer,i,j])
                key=(i,j,round(h/0.02))
                item=(rank,(float(pos[n,0]),float(pos[n,1]),h),layer)
                if key not in choices or rank < choices[key][0]:
                    choices[key]=item
        ordered=sorted(choices.values())
        observations=[]
        if ordered:
            # Keep comparable native-cost/standoff positions, then spread
            # alternatives over the cluster instead of taking three tied
            # cells from the same corner of a large room.
            preferred=[item for item in ordered if item[0] <= ordered[0][0]+0.25]
            observations=[(preferred[0][1],preferred[0][2])]
            coords=np.asarray([item[1] for item in preferred])
            for _ in range(2):
                separation=np.linalg.norm(coords[:,None,:]-np.asarray([p for p,_ in observations])[None,:,:],axis=2).min(axis=1)
                best=int(np.argmax(separation))
                if separation[best] < 0.5:
                    break
                observations.append((preferred[best][1],preferred[best][2]))
        output.append(dict(id=number+1,frontier=xyz[representative].tolist(),
                           cell=[int(cells[representative,1]),int(cells[representative,2])],
                           gain=len(cluster)*resolution,cells=len(cluster),
                           observations=[list(p) for p,_ in observations],
                           display=xyz[np.linspace(0,len(xyz)-1,min(len(xyz),256),dtype=int)].tolist()))
    return dict(frontier_cells=len(nodes),clusters=output,
                known_surface_cells=int(potential.sum()),safe_cells=int(safe.sum()),
                no_observation_clusters=sum(not c['observations'] for c in output))
