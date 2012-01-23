classdef PointCloudSet < handle
    properties
        data_dir;
        num_frames;
        depth_files;
        rgb_files;
        num_frames;
    end
    
    methods
        function obj = PointCloudSet(data_dir)
            %Read list of depth and rgb images.
            obj.data_dir = data_dir;
            obj.depth_files = dir(fullfile(data_dir, 'depth', '*.png'));
            obj.rgb_files = dir(fullfile(data_dir, 'rgb', '*.png'));
            obj.num_frames = numel(obj.depth_files);
        end
        
        function B = subsref(obj,s)
            % Allows access like "pcs{1}"
            if s.type == '{}'
                idx = s.subs{1};
                depthpath = fullfile(obj.data_dir, 'depth', obj.depth_files(idx).name);
                rgbpath = fullfile(obj.data_dir, 'rgb', obj.rgb_files(idx).name);
                B = PointCloud(depthpath, rgbpath);
            else 
                B = builtin('subsref',obj, s);
            end
        end
        
        
    end
end