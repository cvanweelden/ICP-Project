import os
import glob
import shutil

for f in glob.glob('./frame*.ply'):
    i = int(os.path.splitext(os.path.basename(f))[0][5:])
    n = 'frame%04d.ply'%i
    print "renaming %s to %s"%(f, n)
    shutil.move(f, n)
