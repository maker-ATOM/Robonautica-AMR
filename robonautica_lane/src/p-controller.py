import math

def slope_diff(lanes):
    slope_diff_sum = 0

    for lane in lanes:
        x1, y1, x2, y2 = lane
        slope = math.atan2(y2 - y1, x2 - x1)
        slope_diff_sum += slope

    return (slope_diff_sum - math.pi * len(lanes)) % (2 * math.pi) - math.pi


linies = [[0,0,0.9,1],[3,0,2,1]]

print(slope_diff(linies))


