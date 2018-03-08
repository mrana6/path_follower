import h5py
import os
import sys

if __name__ == "__main__":
    inFilename = sys.argv[1]
    outFilename = sys.argv[2]
    delim = sys.argv[3]
    f = h5py.File(inFilename, 'r')
    #   collection 0
    #       _robot_limb_right_enpoint_state 1
    #           pose 1
    #               orientation 0
    #               position 1
    numDataPoints = len(f[list(f.keys())[0]].values()[1].values()[1].values()[1].values()[0])
    with open(outFilename, 'w') as out:
        for i in range(0, numDataPoints):
            # t
            out.write(str(i))
            # x
            out.write(delim)
            out.write(str(f[list(f.keys())[0]].values()[1].values()[1].values()[1].values()[0][i][0]))
            # y
            out.write(delim)
            out.write(str(f[list(f.keys())[0]].values()[1].values()[1].values()[1].values()[1][i][0]))
            # z
            out.write(delim)
            out.write(str(f[list(f.keys())[0]].values()[1].values()[1].values()[1].values()[2][i][0]))
            # x
            out.write(delim)
            out.write(str(f[list(f.keys())[0]].values()[1].values()[1].values()[0].values()[1][i][0]))
            # y
            out.write(delim)
            out.write(str(f[list(f.keys())[0]].values()[1].values()[1].values()[0].values()[2][i][0]))
            # z
            out.write(delim)
            out.write(str(f[list(f.keys())[0]].values()[1].values()[1].values()[0].values()[3][i][0]))
            # w
            out.write(delim)
            out.write(str(f[list(f.keys())[0]].values()[1].values()[1].values()[0].values()[0][i][0])) # note in the h5 it is wxyz i have xyzw
            out.write('\n')
    out.close()
