classdef PointCloudSet < handle
    properties
        data_dir;
        num_frames;
        depth_files;
        rgb_files;
        poses;
    end
    
    methods
        function obj = PointCloudSet(data_dir)
            %Read list of depth and rgb images.
            obj.data_dir = data_dir;
            obj.depth_files = dir(fullfile(data_dir, 'depth', '*.png'));
            obj.rgb_files = dir(fullfile(data_dir, 'rgb', '*.png'));
            obj.num_frames = numel(obj.depth_files);
            if obj.num_frames == 0
                disp('No frames found in dir.');
            end
            ps = cell2mat(textscan(fopen(fullfile(data_dir, 'groundtruth.txt')), ...
                '%f %f %f %f %f %f %f %f', 'HeaderLines',3));
            
            for i=1:obj.num_frames
                [~,timestamp,~] = fileparts(obj.depth_files(i).name);
                timestamp = str2double(timestamp);
                [~,minidx] = min(abs(ps(:,1) - timestamp));
                obj.poses(i,:) = ps(minidx,[5 6 7 8 2 3 4]);
            end
            
        end
        
        function pointcloud = cloud(obj,idx)
            % Allows access like pcs.cloud(i) to fetch a pointcloud.
            depthpath = fullfile(obj.data_dir, 'depth', obj.depth_files(idx).name);
            rgbpath = fullfile(obj.data_dir, 'rgb', obj.rgb_files(idx).name);
            pointcloud = PointCloud(depthpath, rgbpath);
        end
        
        function qt = relpose(obj, to, from)
            % Compute relative pose between two given frame indices.
            if nargin < 3
                from = to - 1;
            end
            if from == 0
                qt = obj.poses(1,:);
                return
            end
            q1 = obj.poses(from,1:4);
            t1 = obj.poses(from,5:7);
            q2 = obj.poses(to,1:4);
            t2 = obj.poses(to,5:7);
            
            dq = quatmultiply(quatinv(q1), q2);
            dt = t2 - quatrotate(dq, t1);
            
            % assert(all(abs(q2 - quatmultiply(q1, dq)) < 1e-5));
            % assert(all(abs(t2 - (quatrotate(dq, t1) + dt)) < 1e-9));
            % assert(all(abs(rigid_multiply(qt, [q1 t1]) - [q2 t2]) < 1e-5));
            qt = [dq dt];
            

        end
            
        
    end
end