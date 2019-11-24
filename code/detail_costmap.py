import numpy as np

import try_interpolation as map_interpolation

world_file = open("worlds/FVD_lunar_surface_V4_demo1.wbt","r")
lunarmap = map_interpolation.LunarMap()

x, y, z = lunarmap.change_resolution(40)

found_rock_flag = 0
rock_size = 0

rocks = []

for i, line in enumerate(world_file):
    if(line.rstrip() == "Rock10cmCustom {" or line.rstrip() == "Rock17cmCustom {" or line.rstrip() == "Rock40cmCustom {" or line.rstrip() == "Rock100cmCustom {"):
        found_rock_flag = i
        if(line.rstrip() == "Rock10cmCustom {"):
            rock_size = 100 * 1e-3
        elif(line.rstrip() == "Rock17cmCustom {"):
            rock_size = 170 * 1e-3
        elif(line.rstrip() == "Rock40cmCustom {"):
            rock_size = 400 * 1e-3
        elif(line.rstrip() == "Rock100cmCustom {"):
            rock_size = 1000 * 1e-3
    elif(found_rock_flag and i == found_rock_flag+1):
        translations = line.split(" ")
        x_rock = float(translations[3].strip())# + lunarmap.side_len/2
        y_rock = float(translations[5].strip())# + lunarmap.side_len/2
        z_rock = float(translations[4].strip())
        # import pdb; pdb.set_trace()
        x_rock_meters = x_rock
        y_rock_meters = y_rock
        rocks.append([x_rock_meters, y_rock_meters, rock_size])
        # x_rock_map = floor(x_rock*2)
        # y_rock_map = floor(y_rock*2)
        # z[x_rock_map, y_rock_map] += rock_size
        found_rock_flag = 0

np.save("data/rock_info.npy",np.array(rocks))
