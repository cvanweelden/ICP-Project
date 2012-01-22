classdef PointCloudIterator < handle
    properties
        data_dir;
        num_frames;
        depth_files;
        rgb_files;
        index;
        pointcloud;
        rgb_values;
    end
    methods
        function obj = PointCloudIterator(data_dir)
            %Read list of depth and rgb images.
            obj.data_dir = data_dir;
            obj.depth_files = dir([data_dir filesep 'depth' filesep '*.png']);
            obj.rgb_files = dir([data_dir filesep 'rgb' filesep '*.png']);
            obj.index = 0;
            obj.num_frames = length(obj.depth_files);
        end
        function bool = has_next(obj)
            bool = obj.index < obj.num_frames;
        end
        function next(obj)
            obj.index = obj.index + 1;
            depth_frame = imread([obj.data_dir filesep 'depth' filesep obj.depth_files(obj.index).name]);
            rgb_frame = imread([obj.data_dir filesep 'rgb' filesep obj.rgb_files(obj.index).name]);
            [obj.pointcloud obj.rgb_values] = depthmap2pointcloud(depth_frame, rgb_frame);
        end
    end
end