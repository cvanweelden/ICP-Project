% PointCloud class analogous to PCL's PointCloud class.

classdef PointCloud < handle
    properties
        n;
        xyz;
        rgb;
        normals;
        timestamp;
    end
    methods
        function obj = PointCloud(depthpath, rgbpath)
            % Constructor
            if nargin >= 2
                obj.from_frame(imread(depthpath), imread(rgbpath));
                % Set timestamp
                [~, obj.timestamp, ~] = fileparts(depthpath);
            end
        end
        
        function pc = horzcat(obj, other)
            % Horizontal concatenation: to enable "pc = [pc1 pc2];"
            pc = PointCloud();
            pc.n = obj.n + other.n;
            pc.xyz = [obj.xyz other.xyz];
            pc.rgb = [obj.rgb other.rgb];
        end
        
        function pc = copy(obj)
            % We need to implement copy explicitly b/c this is a handle
            pc = PointCloud();
            pc.n = obj.n;
            pc.xyz = obj.xyz;
            pc.rgb = obj.rgb;
            pc.normals = obj.normals;
            pc.timestamp = obj.timestamp;
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
                obj.rgb = reshape(rgbim, [], 3)';
            else
                obj.rgb = zeros(0,obj.n);
            end
            
            % Initialize empty normals
            obj.normals = zeros(0, obj.n);
            
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
            obj.computenormals();
        end
        
        function apply_qt(obj, qt)
            % Apply a quaternion+translation transformation
            obj.xyz = rigid_transform(qt, obj.xyz);
            obj.computenormals();
        end
        
        function subsample(obj, k)
            % Subsample randomly to k points
            
            idx = randsample(obj.n, k);
            obj.rgb = obj.rgb(:,idx);
            obj.xyz = obj.xyz(:,idx);
            obj.n = k;
        end
                
        function computenormals(obj, k)
            % Compute the normals using the eigenvector corresponding
            % to the smallest eigenvalue of the pca of the covariance-
            % matrix of the closest points.
            % As in the Gen-ICP paper by Segal, Haehnel & Thrun.
            if nargin < 2
                % Use 20 neighbours by default
                k = 20;
            end
            obj.normals = [];
            k = min(obj.n, k);
            tree = kdtree_build(obj.xyz');
            for i=1:obj.n
                p = obj.xyz(:,i);
                nearest = kdtree_k_nearest_neighbors(tree, p', k);
                nearest = obj.xyz(:,nearest);
                centered = nearest - repmat(p, 1, k);
                coeffs = princomp(cov(centered'));
                normal = coeffs(:,end);
                obj.normals(:,i) = normal/norm(normal);
            end
            kdtree_delete(tree);
        end
                
        function write(obj, filename)
            % Write a PLY file containing this point cloud
    
            f = fopen(filename, 'w');
    
            fprintf(f, 'ply\n');
            fprintf(f, 'format ascii 1.0\n');
            fprintf(f, 'element vertex %d\n', obj.n);
            fprintf(f, 'property float x\n');
            fprintf(f, 'property float y\n');
            fprintf(f, 'property float z\n');
            fprintf(f, 'property uchar red\n');
            fprintf(f, 'property uchar green\n');
            fprintf(f, 'property uchar blue\n');
            fprintf(f, 'property uchar alpha\n');
            fprintf(f, 'end_header\n');

            for i=1:obj.n
                fprintf(f, '%f %f %f %d %d %d 0\n', obj.xyz(:,i), obj.rgb(:,i));
            end

            fclose(f);
        end
        

    end
end