import numpy as np

import try_interpolation as map_interpolation

world_file = open("worlds/custom_lunar.wbt","r")
lunarmap = map_interpolation.LunarMap()

x, y, z = lunarmap.change_resolution(40)

found_rock_flag = 0
rock_size = 0

for i, line in enumerate(world_file):
    if(line.rstrip() == "Rock10cmCustom {"):
        found_rock_flag = i
        rock_size = 10 * 1e-3
    elif(found_rock_flag and i == found_rock_flag+3):
        translations = line.split(" ")
        x_rock = float(translations[3].strip()) + lunarmap.side_len/2
        y_rock = float(translations[4].strip()) + lunarmap.side_len/2
        x_rock_map = floor(x_rock*2)
        y_rock_map = floor(y_rock*2)
        z[x_rock_map, y_rock_map] += rock_size
        found_rock_flag = 0
