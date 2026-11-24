"""A cut's sparse cells, terrain costs and incremental frontier index."""

import numpy as np

from .fixed_layer_map import CELL_DTYPE, FLOATS, different


class TomographicSlice:
    def __init__(self, mapping, index):
        self.index = index
        self.settings = mapping.settings
        self.height = self.settings.origin_z + index*self.settings.slice_dh
        self.blocks = {}
        self._block_counts = {}
        self.counts = {}
        self.frontiers = mapping.frontier_index.add_slice(index)

    def allocate(self, tile):
        size = self.settings.block_size
        block = np.zeros((size, size), dtype=CELL_DTYPE)
        for name in FLOATS:
            block[name] = np.nan
        for name in ('passage_cost', 'relief_cost', 'terrain_cost',
                     'inflated_terrain_cost', 'terrain_inflation_reference'):
            block[name] = -1
        block['unknown'] = block['hard'] = block['unknown_hard'] = True
        self.blocks[tile] = block
        return block

    @staticmethod
    def _commit_surface(row, name, surface):
        height,identity,mature=surface
        field=name.lower()
        changed=(different(float(row[name]),height) or row[field+'_mature']!=mature or
                 row[field+'_surface_id']!=identity)
        row[name]=height
        row[field+'_mature']=mature
        row[field+'_surface_id']=identity
        return changed

    def update_geometry(self, evidence, kernel=None):
        if kernel is not None:
            return kernel(self.blocks,self.settings.block_size,evidence)
        size = self.settings.block_size
        ground_changed, base_changed = set(), set()
        for (x,y), (ground,ceiling) in evidence.items():
            row = self.blocks[x//size,y//size][x%size,y%size]
            row['measured'] = True
            g = self._commit_surface(row, 'G', ground)
            c = self._commit_surface(row, 'C', ceiling)
            if g:
                ground_changed.add((x,y))
            if g or c:
                base_changed.add((x,y))
        return base_changed, ground_changed

    def update_costs(self, mapping, changes):
        base, ground = changes.get(self.index,(set(),set()))
        neighbors = (self.index-1,self.index+1)
        neighbor_changes = set().union(*(changes.get(k,((),()))[1] for k in neighbors))
        if not (base or ground or neighbor_changes):
            return base, set(), {}
        settings = self.settings
        adjacent = [mapping.layers[k].blocks if k in mapping.layers else {} for k in neighbors]
        if mapping._cost_kernel is not None:
            result = mapping._cost_kernel(self.blocks, settings.block_size, base, ground,
                                          vars(settings), *adjacent, neighbor_changes)
            for tile,counts in result['counts'].items():
                old = self._block_counts.get(tile, {})
                for name,count in counts.items():
                    self.counts[name] = self.counts.get(name, 0)+count-old.get(name, 0)
                self._block_counts[tile] = counts
            return base, result['changed'], result['work']

        from .map_costs import update_costs
        # Serial reference path. Its block keys retain the map-wide format.
        class CostView:
            def __init__(self, layer):
                self.settings = layer.settings
                self.blocks = {(layer.index,*tile): block for tile,block in layer.blocks.items()}
                self._block_counts = layer._block_counts
                self.counts = layer.counts

            def cell(self, key):
                k,x,y = key
                block = self.blocks.get((k,x//settings.block_size,y//settings.block_size))
                if block is not None:
                    return block[x%settings.block_size,y%settings.block_size]
                return None

            def nearby_height(self, key, reference):
                _,x,y=key
                return mapping.nearby_cut_height(self_layer,x,y,reference)

        self_layer=self.index
        view = CostView(self)
        changed = update_costs(view, {(self.index,*xy) for xy in base},
                              {(self.index,*xy) for xy in ground},
                              {(self.index,*xy) for xy in neighbor_changes})
        return base, {key[1:] for key in changed}, view.last_work
