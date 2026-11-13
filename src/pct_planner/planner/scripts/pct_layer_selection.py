"""PCT endpoint selection shared by dense and online tomograms."""
from math import isfinite


def rank_layers(layers, target_height, tolerance, read_height, read_cost):
    """Order height-compatible layers by cost, keeping input order on ties.

    The map reader returns None for a layer whose cost is ineligible.
    """
    if not isfinite(target_height):
        return []
    matches = []
    for layer in layers:
        height = read_height(layer)
        if not isfinite(height) or abs(height - target_height) > tolerance:
            continue
        cost = read_cost(layer)
        if cost is not None and isfinite(cost):
            matches.append((cost, layer))
    matches.sort(key=lambda match: match[0])
    return [layer for _, layer in matches]
