"""Level-area evidence shared by platform confirmation and ordinary exploration."""
import math


def platform_cells(settings, center, supported, minimum_area=None):
    area=settings.landing_min_area if minimum_area is None else minimum_area
    required=math.ceil(area/settings.resolution**2-1e-7)
    extent=math.ceil(max(2*math.sqrt(area),2*(settings.robot_radius+settings.safety_margin)) /
                     settings.resolution)+1
    if not supported(*center):return set()
    seen={center};pending=[center];connected=set()
    while pending:
        x,y=pending.pop();connected.add((x,y))
        if len(connected)>=required:return connected
        for dx,dy in ((-1,0),(1,0),(0,-1),(0,1)):
            other=x+dx,y+dy
            if other in seen or max(abs(other[0]-center[0]),abs(other[1]-center[1]))>extent:continue
            seen.add(other)
            if supported(*other):pending.append(other)
    return connected
