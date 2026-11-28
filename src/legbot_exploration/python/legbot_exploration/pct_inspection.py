"""Full display messages from sparse blocks; buffers are not planning maps."""
import numpy as np
import time

SURFACE_DTYPE = np.dtype([(name, '<f4') for name in
    ('x', 'y', 'z', 'rgb', 'cost', 'ceiling_z', 'clearance')]+[
    ('source', '<u4'), ('flags', '<u4'), ('slice_index', '<u4')]+[
    (name, '<f4') for name in ('passage_cost', 'relief_cost', 'terrain_cost', 'inflated_terrain_cost',
                             'node_z', 'C_clear')])
POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
# Keep the 64-byte surface wire format and existing evidence bits unchanged.
# flags[8:16] records the common planning-state reason.
NAVIGATION_REASONS=('ready','missing_support','known_obstacle','known_obstacle_footprint',
                    'missing_footprint','insufficient_clearance','terrain_limit','unassessed')


def surface_chunk(m, key):
    s=m.settings;k,bx,by=key;block=m.blocks.get(key)
    if block is None:return np.empty(0,dtype=SURFACE_DTYPE)
    i,j=np.nonzero(np.isfinite(block['G']))
    if not len(i):return np.empty(0,dtype=SURFACE_DTYPE)
    row=block[i,j];chunk=np.empty(len(i),dtype=SURFACE_DTYPE)
    states=[m.planning_state((k,bx*s.block_size+int(x),by*s.block_size+int(y))) for x,y in zip(i,j)]
    cost=np.asarray([state[0] for state in states],dtype='<f4')
    reasons=np.asarray([NAVIGATION_REASONS.index(state[1]) for state in states],dtype='<u4')
    rgb=np.where(np.isin(reasons,(2,3,5)),0xff3030,np.where(np.isin(reasons,(1,4,7)),0x4070d0,
        np.where(cost>20,0x808080,0x20c860))).astype('<u4')
    chunk['x']=s.origin_x+(bx*s.block_size+i)*s.resolution
    chunk['y']=s.origin_y+(by*s.block_size+j)*s.resolution
    chunk['z'],chunk['rgb'],chunk['cost']=row['G'],rgb.view('<f4'),cost
    chunk['ceiling_z']=row['C']
    chunk['clearance']=np.where(np.isfinite(row['C']),row['C'],row['C_clear'])-row['G']
    chunk['source']=1
    chunk['flags']=sum(row[name].astype('<u4')<<bit for bit,name in enumerate(
        ('solid','unknown','solid_hard','unknown_hard')))
    chunk['flags']|=np.isfinite(row['C']).astype('<u4')<<4
    chunk['flags']|=row['g_mature'].astype('<u4')<<5
    chunk['flags']|=row['c_mature'].astype('<u4')<<6
    chunk['flags']|=reasons<<8
    chunk['slice_index']=np.uint32(k & 0xffffffff)
    for name in ('passage_cost','relief_cost','terrain_cost','inflated_terrain_cost','node_z','C_clear'):
        chunk[name]=row[name]
    chunk.flags.writeable=False
    return chunk


def capture(backend):
    """Locked phase: encode changed immutable chunks, never concatenate a map.

    C++ retains native shared buffers and renders without the map lock or GIL.
    Compatibility inspect() remains available for standalone diagnostics.
    """
    m,s=backend.map,backend.map.settings
    reset=getattr(backend,'_export_epoch',None)!=m.export_epoch
    blocks=set(m.blocks) if reset else m.export_dirty_blocks.copy()
    columns=set(m.columns) if reset else m.export_dirty_columns.copy()
    surfaces=[(key,surface_chunk(m,key).tobytes()) for key in sorted(blocks)]
    returns=[]
    for x,y in sorted(columns):
        rows=[(s.origin_x+x*s.resolution,s.origin_y+y*s.resolution,e[1],0xff5050)
              for e in m.columns.get((x,y),{}).values()]
        returns.append(((x,y),np.asarray(rows,dtype=POINT_DTYPE).tobytes()))
    m.export_dirty_blocks.difference_update(blocks)
    m.export_dirty_columns.difference_update(columns)
    backend._export_epoch=m.export_epoch
    metadata=dict(backend.metadata,export_changed_blocks=len(blocks),export_changed_columns=len(columns))
    return dict(reset=reset,epoch=m.export_epoch,first_layer=m.first_layer or 0,
                surfaces=surfaces,returns=returns,metadata=metadata,
                changed_blocks=len(blocks),changed_columns=len(columns))


def inspect(backend, base, options):
    show, max_display, preview, radius, min_gain, max_candidates, max_nodes = options
    m, s = backend.map, backend.map.settings
    result = dict(surfaces=b'', display=b'', returns=b'', ceilings=b'', nodes=[], candidates=[], path=[],
                  preview_state='disabled', metadata=dict(backend.metadata))
    if not show:
        return result
    cache_key = (m.revision, max_display)
    if getattr(backend, '_display_key', None) != cache_key:
        # Only message buffers are assembled here; no second planning map.
        # Vectorize each sparse block instead of interpreting every row in Python.
        chunks=[surface_chunk(m,key).copy() for key in m.blocks]
        for chunk in chunks:
            chunk['slice_index']=(chunk['slice_index'].view('<i4')-m.first_layer).astype('<u4')
        records=np.concatenate(chunks) if chunks else np.empty(0,dtype=SURFACE_DTYPE)
        # Shared G is identical across its slice representations. Keep the
        # complete planner payload, but draw each physical surface once using
        # a traversable representation when admission has one.
        colors=records['rgb'].view('<u4')
        rank=np.where(colors==0x20c860,0,np.where(colors==0x808080,1,np.where(colors==0x4070d0,2,3)))
        order=np.lexsort((records['cost'],rank,records['z'],records['y'],records['x']))
        ordered=records[order]
        unique=np.r_[True,(ordered['x'][1:]!=ordered['x'][:-1])|
                          (ordered['y'][1:]!=ordered['y'][:-1])|
                          (ordered['z'][1:]!=ordered['z'][:-1])] if len(ordered) else np.zeros(0,dtype=bool)
        display=ordered[unique]
        stride = max(1, int(np.ceil(len(display)/max_display)))
        ceiling = records[np.isfinite(records['ceiling_z'])][::stride]
        cloud = np.empty(len(ceiling), dtype=POINT_DTYPE)
        for name in ('x', 'y'):
            cloud[name] = ceiling[name]
        cloud['z'], cloud['rgb'] = ceiling['ceiling_z'], 0xffb030
        backend._display_payload = dict(surfaces=records.tobytes(), display=display[::stride].tobytes(), ceilings=cloud.tobytes())
        backend._display_key = cache_key
    result.update(backend._display_payload)
    # New out-of-band surfaces update the measured returns independently of display caching.
    if getattr(backend,'_returns_revision',None)!=m.evidence_revision:
        hits = [(s.origin_x+x*s.resolution, s.origin_y+y*s.resolution, e[1], 0xff5050)
                for (x, y), column in m.columns.items() for e in column.values()]
        backend._returns_payload=np.asarray(hits,dtype=POINT_DTYPE).tobytes()
        backend._returns_revision=m.evidence_revision
    result['returns']=backend._returns_payload
    records = np.frombuffer(result['surfaces'], dtype=SURFACE_DTYPE)
    result['metadata']['display_surfaces'] = len(records)
    if not preview:
        return result
    # Diagnostic preview may reuse one committed map/cell. Actual execution
    # planning always uses the exact pose and never uses this preview cache.
    preview_key=(m.revision,tuple(m.indices(base)),round(base[2]/s.resolution),
                 radius,min_gain,max_candidates,max_nodes)
    if getattr(backend,'_preview_key',None)==preview_key:
        result.update(backend._preview_payload)
        return result
    started=time.monotonic()
    _preview(backend,base,records,result,radius,min_gain,max_candidates,max_nodes)
    backend._preview_key=preview_key
    backend._preview_payload={key:result[key] for key in ('nodes','candidates','path','preview_state')}
    result['metadata']['preview_ms']=1000*(time.monotonic()-started)
    return result


def _preview(backend,base,records,result,radius,min_gain,max_candidates,max_nodes):
    m,s=backend.map,backend.map.settings
    admitted = m.point_safe_check(base, backend.reference_height)
    if not admitted['ready']:
        result['preview_state'] = 'unsafe_start:'+admitted['reason']
        return
    distance = np.hypot(records['x']-base[0], records['y']-base[1])
    nearby = (records['cost'] <= 20) & (records['cost'] >= 0) & (distance <= radius)
    safe = records[nearby]
    stride = max(1, int(np.ceil(len(safe)/max_nodes)))
    result['nodes'] = np.column_stack((safe['x'], safe['y'], safe['z']))[::stride].astype(float).tolist()
    rise = nearby & (records['z'] >= admitted['ground_z']+min_gain)
    candidates = records[rise][np.argsort(distance[rise])]
    chosen = []
    for cell in candidates:
        point = np.array([cell['x'], cell['y'], cell['z']], dtype=float)
        if any(np.linalg.norm(point-old) < max(s.resolution, .25) for old in chosen):
            continue
        chosen.append(point)
        if len(chosen) >= max_candidates:
            break
    result['candidates'] = [p.tolist() for p in chosen]
    result['preview_state'] = 'no_rising_surface' if not chosen else 'no_usable_ascent_route'
    for point in chosen:
        target = point+[0, 0, backend.reference_height]
        reason, path, _ = backend.plan(b'', backend.version, base, target)
        if reason == 'ok':
            result['path'] = [[p[0], p[1], p[2]-backend.reference_height] for p in path]
            result['preview_state'] = 'ok'
            break
