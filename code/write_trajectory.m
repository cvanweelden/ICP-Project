function write_trajectory( results, filename )
%WRITE_TRAJECTORY Write the trajectory in RESULTS to FILENAME according to
%the file format from:
%http://cvpr.in.tum.de/data/datasets/rgbd-dataset/file_formats

f = fopen(filename, 'w');

for i = 1:size(results.pose, 2)
    fprintf(f, '%s %f %f %f %f %f %f %f\n', results.timestamp{i}, results.pose{i}(5:7), results.pose{i}(1:4)); 
end

fclose(f);

end

