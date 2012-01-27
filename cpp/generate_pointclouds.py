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

def generate_pointcloud(rgb_file,depth_file,ply_file):
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
            points.append("%f %f %f %d %d %d 0\n"%(X,Y,Z,color[0],color[1],color[2]))
    file = open(ply_file,"w")
    file.write('''ply
format ascii 1.0
element vertex %d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar alpha
end_header
%s
'''%(len(points),"".join(points)))
    file.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script reads a registered pair of color and depth images and generates a colored 3D point cloud in the
    PLY format. 
    ''')
    parser.add_argument('--rgb', '-r', help='input color image (format: png)')
    parser.add_argument('--depth', '-d', help='input depth image (format: png)')
    parser.add_argument('--output', '-o', help='output ply file or folder')
    parser.add_argument('--folder', '-f', help='input folder')
    parser.add_argument('-n', help='max frames to write', default=10)
    args = parser.parse_args()
    if args.rgb and args.depth:
        generate_pointcloud(args.rgb,args.depth,args.output)
    # Process an entire folder
    if args.folder:
        if not os.path.exists(args.output):
            os.mkdir(args.output)
        rgbfiles = glob.glob(os.path.join(args.folder,'rgb','*.png'))
        depthfiles = glob.glob(os.path.join(args.folder,'depth','*.png'))
        for i,(rgb, d) in enumerate(zip(rgbfiles, depthfiles)):
            if n > 0 && i >= n:
                break
            generate_pointcloud(rgb, d, os.path.join(args.output, 'frame%03d.ply'%i))
            
    