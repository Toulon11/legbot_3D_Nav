"""Incremental measured-support cache, independent of stair shapes or flights."""
from bisect import bisect_left, bisect_right, insort
from collections import Counter
import math


class SupportTargets:
    def __init__(self, mapping):
        self.map=mapping
        self.columns={}
        self.tiles={}
        self.by_height={}
        self.heights=[]
        self.revision=0

    def update(self, changed):
        size=self.map.settings.block_size
        for xy in {(x,y) for _,x,y in changed}:
            current={}
            for k,row in self.map.layers_at(*xy):
                cut=self.map.settings.origin_z+k*self.map.settings.slice_dh
                if (self.map._covers_cut(row,cut) and not row['solid'] and 0<=row['terrain_cost']<=20 and
                        self.map.surface_observation.is_ground_source(xy,int(row['g_surface_id']))):
                    h,sid=float(row['G']),int(row['g_surface_id'])
                    current.setdefault((h,sid),(k,h,sid,float(row['terrain_cost'])))
            rows=tuple(sorted(current.values(),key=lambda r:(r[1],r[2])))
            previous=self.columns.get(xy,())
            if previous==rows:
                continue
            self.revision+=1
            tile=(xy[0]//size,xy[1]//size)
            old_bins={math.floor(row[1]/.08) for row in previous}
            new_bins={math.floor(row[1]/.08) for row in rows}
            for band in old_bins-new_bins:
                counter=self.by_height[band];counter[tile]-=1
                if not counter[tile]:del counter[tile]
                if not counter:
                    del self.by_height[band]
                    self.heights.pop(bisect_left(self.heights,band))
            for band in new_bins-old_bins:
                if band not in self.by_height:
                    self.by_height[band]=Counter();insort(self.heights,band)
                self.by_height[band][tile]+=1
            if rows:
                self.columns[xy]=rows
                self.tiles.setdefault(tile,{})[xy]=rows
            else:
                self.columns.pop(xy,None)
                self.tiles[tile].pop(xy,None)
                if not self.tiles[tile]:del self.tiles[tile]

    def candidates(self, base, low, high, minimum_distance=0., radius=None, limit=96,prefer_ascent=False):
        """Read nearby cached tiles in a height band, never rebuild from the map."""
        settings=self.map.settings
        start=bisect_left(self.heights,math.floor(low/.08))
        stop=bisect_right(self.heights,math.floor(high/.08))
        tiles=set()
        for band in self.heights[start:stop]:tiles.update(self.by_height[band])
        pitch=settings.block_size*settings.resolution

        def distance(tile):
            return math.hypot(max(settings.origin_x+tile[0]*pitch-base[0],0.,
                                  base[0]-(settings.origin_x+(tile[0]+1)*pitch)),
                              max(settings.origin_y+tile[1]*pitch-base[1],0.,
                                  base[1]-(settings.origin_y+(tile[1]+1)*pitch)))

        choices=[]
        for tile in sorted(tiles,key=lambda t:(distance(t),t)):
            lower=distance(tile)
            if radius is not None and lower>radius:
                break
            if not prefer_ascent and len(choices)>=limit and lower>sorted(choices)[limit-1][0]:
                break
            for (x,y),rows in self.tiles[tile].items():
                px,py=self.map.position((0,x,y))
                d=math.hypot(px-base[0],py-base[1])
                if d<minimum_distance or (radius is not None and d>radius):continue
                for k,h,sid,cost in rows:
                    if low<=h<=high:
                        choices.append((d,cost,-h,(float(px),float(py),h),k))
        if not prefer_ascent:return sorted(choices)[:limit]
        # Dense high rails/risers must not consume the entire bounded pool.
        # Keep nearby representatives from each existing 8 cm height band,
        # then retain the requested high-first execution preference.
        buckets={}
        for row in choices:buckets.setdefault(math.floor(row[3][2]/.08),[]).append(row)
        for rows in buckets.values():rows.sort(key=lambda row:(row[0],row[1],row[2],row[3]))
        selected=[]
        for at in range(limit):
            batch=[buckets[k][at] for k in sorted(buckets,reverse=True) if at<len(buckets[k])]
            if not batch:break
            selected.extend(batch[:limit-len(selected)])
            if len(selected)>=limit:break
        return sorted(selected,key=lambda row:(row[2],row[0],row[1],row[3]))
