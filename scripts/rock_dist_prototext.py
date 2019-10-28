import sys
import argparse
import numpy as np

class Rock:
    def __init__(self, xlowlimit, xhighlimit, zlowlimit, zhighlimit, all_y=None):
        self.xlowlimit = xlowlimit
        self.xhighlimit = xhighlimit
        self.zlowlimit = zlowlimit
        self.zhighlimit = zhighlimit

    def rock_coordinate(self, x, z):
        x = (self.xhighlimit - self.xlowlimit) * x + self.xlowlimit
        z = (self.zhighlimit - self.zlowlimit) * z + self.zlowlimit
        y = 0
        return str(x)+" "+str(y)+" "+str(z)

    def rock_rotation(self, rot):
        rot = rot * 2 * np.pi
        return "0 -1 0 " + str(rot) 

def parse_arguments():
    # Command-line flags are defined here.
    parser = argparse.ArgumentParser()
    parser.add_argument('--xlowlimit', dest='xlowlimit', type=int, default=-800, help="Lower x limit")
    parser.add_argument('--xhighlimit', dest='xhighlimit', type=int, default=800, help="Higher x limit")
    parser.add_argument('--zlowlimit', dest='zlowlimit', type=int, default=-800, help="Lower z limit")
    parser.add_argument('--zhighlimit', dest='zhighlimit', type=int, default=800, help="Higher z limit")
    parser.add_argument('--numrocks', dest='numrocks', type=int, default=10000, help="Total number of rocks")
    return parser.parse_args()

def main():
    # Parse command-line arguments.
    args = parse_arguments()
    numrocks = args.numrocks

    world_file = open("../worlds/sojourner_test.wbt", "a+")
    remove_line = False

    for line in world_file:
        if(line.rstrip() == "# Rocks go here!"):
            print("Found Start of rocks")
            remove_line = True
        elif(line.rstrip() == "# Rocks end here!"):
            print("Found End of rocks")
            remove_line = False
        # elif(remove_line):
            

    rock_file = open("rock_write.txt","w") 

    rockit = Rock(args.xlowlimit, args.xhighlimit, args.zlowlimit, args.zhighlimit)

    x = np.random.random_sample((numrocks,))
    z = np.random.random_sample((numrocks,))
    rot = np.random.random_sample((numrocks,))
    # import pdb; pdb.set_trace()
    for i in range(numrocks):
        rock_file.write("Rock10cmCustom {\n  translation " + rockit.rock_coordinate(x[i], z[i])+ "\n  rotation " + rockit.rock_rotation(rot[i])+"\n  name \"rock 10 cm("+str(i)+")\"\n}")


if __name__ == '__main__':
    main()