% PointCloud class analogous to PCL's PointCloud class.

classdef PointCloud < handle
    properties
        n;
        xyz;
        rgb;
        has_rgb;
    end
    methods
        function obj = PointCloud(depthpath, rgbpath)
            % Constructor
            if nargin >= 2
                obj.from_frame(imread(depthpath), imread(rgbpath));
            end
        end
        
        function pc = horzcat(obj, other)
            % Horizontal concatenation: to enable "pc = [pc1 pc2];"
            pc = PointCloud();
            pc.n = obj.n + other.n;
            pc.xyz = [obj.xyz other.xyz];
            if obj.has_rgb && other.has_rgb
                pc.rgb = [obj.rgb other.rgb];
                pc.has_rgb = true;
            end
        end
        
        
        function from_frame(obj, depth, varargin)
            % Converts a depthmap (+rgb image) to a pointcloud
             
            % Parse input arguments
            p = inputParser;
            p.addOptional('rgb', [], @(x)isnumeric(x));
            p.addParamValue('CamVars', struct('fx',525,'fy',525,'cx',319.5,'cy',239.5,'ds',1/5000));
            p.parse(varargin{:});
            rgbim = p.Results.rgb;
            camvar = p.Results.CamVars;
                        
            % Straighten the XYZ coordinates using the camera parameters
            [I J] = meshgrid(1:size(depth,2), 1:size(depth,1));
            Z = double(depth) * camvar.ds;
            X = (I - camvar.cx) .* Z / camvar.fx;
            Y = (J - camvar.cy) .* Z / camvar.fy;
            obj.xyz = [X(:)'; Y(:)'; Z(:)'];
            
            % Set num_points
            obj.n = size(obj.xyz,2);
            
            % Load RGB, if provided
            if numel(rgbim) > 0
                obj.has_rgb = true;
                obj.rgb = reshape(rgbim, [], 3)';
            end
            
            size(obj.xyz)
            % Filter invalid pixels
            mask = find(Z ~= 0);
            obj.xyz = obj.xyz(:,mask);
            obj.rgb = obj.rgb(:,mask);
            
            % Set num_points
            obj.n = size(obj.xyz,2);
                
        end
        
        function apply_matrix(obj, T)
            % Apply a (homogeneous) transformation matrix
            obj.xyz = T * [obj.xyz; ones(1, obj.n)];
            obj.xyz = obj.xyz(1:3,:);
        end
        
        function apply_qt(obj, q, t)
            % Apply a quaternion+translation transformation
            % TODO
        end
        
        function subsample(obj, k)
            % Subsample randomly to k points
            
            idx = randsample(size(in,2),k);
            obj.rgb = obj.rgb(:,idx);
            obj.xyz = obj.xyz(:,idx);
        end
        
        function write(obj, filename)
            % Write a PLY file containing this point cloud
    
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

            for i=1:obj.width
                fprintf(f, '%f %f %f %d %d %d 0\n', XYZ(:,i), RGB(:,i));
            end

            fclose(f);
        end
        

    end
end