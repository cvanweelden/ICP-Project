function [ output_args ] = ply( XYZ, RGB, filename )
%PLY Convert and save an xyz/rgb image to a ply file.

    % Convert the input files to pixels if needed
    if ndims(XYZ) > 2
        XYZ = im2pixels(XYZ);
    end
    if ndims(RGB) > 2
        RGB = im2pixels(RGB);
    end
    
    n = size(XYZ,2);
    
    f = fopen(filename, 'w');
    
    fprintf(f, 'ply\n');
    fprintf(f, 'format ascii 1.0\n');
    fprintf(f, 'element vertex %d\n',n);
    fprintf(f, 'property float x\n');
    fprintf(f, 'property float y\n');
    fprintf(f, 'property float z\n');
    fprintf(f, 'property uchar red\n');
    fprintf(f, 'property uchar green\n');
    fprintf(f, 'property uchar blue\n');
    fprintf(f, 'property uchar alpha\n');
    fprintf(f, 'end_header\n');
    
    for i=1:n
        fprintf(f, '%f %f %f %d %d %d 0\n', XYZ(:,i), RGB(:,i));
    end
    
    fclose(f);
    
end

