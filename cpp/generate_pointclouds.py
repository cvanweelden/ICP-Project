#!/usr/bin/python
#
# the resulting .ply file can be viewed for example with meshlab
# sudo apt-get install meshlab

import argparse
import sys
import os
import glob
from PIL import Image

focalLength = 525.0
centerX = 319.5
centerY = 239.5
scalingFactor = 5000.0

def generate_pointclouds(rgb_file,depth_file,fn,format='ply'):
    print "Creating points for %s + %s" % (rgb_file, depth_file)
    rgb = Image.open(rgb_file)
    depth = Image.open(depth_file)
    
    if rgb.size != depth.size:
        raise Exception("Color and depth image do not have the same resolution.")
    if rgb.mode != "RGB":
        raise Exception("Color image is not in RGB format")
    if depth.mode != "I":
        raise Exception("Depth image is not in intensity format")

    points = []    
    for v in range(rgb.size[1]):
        for u in range(rgb.size[0]):
            color = rgb.getpixel((u,v))
            Z = depth.getpixel((u,v)) / scalingFactor
            if Z==0: continue
            X = (u - centerX) * Z / focalLength
            Y = (v - centerY) * Z / focalLength
            points.append((X,Y,Z,color[0],color[1],color[2]))
    
    writer = globals()['write_'+format]
    writer(points, rgb.size[0], rgb.size[1], fn)

    
def write_ply(points, width, height, fn):
    (base, _) = os.path.splitext(fn)
    fn = base+'.ply'
    print "Writing to %s"%(fn)
    points_str = "".join("%f %f %f %d %d %d 0\n"%p for p in points)
    file = open(fn,"w")
    file.write(("ply\n"
        "format ascii 1.0\n"
        "element vertex %d\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "property uchar alpha\n"
        "end_header\n"
        "%s")%(len(points),points_str))
    file.close()

    
def write_pcd(points, width, height, fn):
    (base, _) = os.path.splitext(fn)
    fn = base+'.pcd'
    print "Writing to %s"%(fn)
    points_str = "".join("%f %f %f %d %d %d\n"%p for p in points)
    file = open(fn,"w")
    file.write(("# .PCD v.7 - Point Cloud Data file format\n"
        "VERSION .7\n"
        "FIELDS x y z r g b\n"
        "SIZE 4 4 4 4 4 4\n"
        "TYPE F F F U U U\n"
        "COUNT 1 1 1 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT %d\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA ascii\n"
        "%s")%(width, height, len(points),points_str))
    file.close()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script reads a registered pair of color and depth images and generates a colored 3D point cloud in the
    PLY format. 
    ''')
    parser.add_argument('--rgb', '-r', help='input color image (format: png)')
    parser.add_argument('--depth', '-d', help='input depth image (format: png)')
    parser.add_argument('--output', '-o', help='output file or folder')
    parser.add_argument('--folder', '-f', help='input folder')
    parser.add_argument('--format', '-t', help='output format', default='pcd')
    parser.add_argument('-n', help='max frames to write', type=int, default=10)
    args = parser.parse_args()
    if args.rgb and args.depth:
        generate_pointclouds(args.rgb,args.depth,args.output, format=args.format)
    # Process an entire folder
    if args.folder:
        if not os.path.exists(args.output):
            os.mkdir(args.output)
        rgbfiles = glob.glob(os.path.join(args.folder,'rgb','*.png'))
        depthfiles = glob.glob(os.path.join(args.folder,'depth','*.png'))
        for i,(rgb, d) in enumerate(zip(rgbfiles, depthfiles)):
            if args.n > 0 and i >= args.n:
                break
            generate_pointclouds(rgb, d, os.path.join(args.output, 'frame%03d'%i), format=args.format)
            
    