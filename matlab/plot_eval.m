function plot_eval(eval_files, methods, num_frames)
%Plot the mean error and magnitude of the estimated translations for each
%frameskip offset and the acumulated error over NUM_FRAMES frames.
%EVAL_FILES is a cell array with filepaths for the files generated with
%evaluation.cpp and METHODS is a cell array with the method names.

num_methods = length(methods);

%Compute the errors
for i = 1:num_methods
    [eq{i} et{i}] = mean_error_at_offset(eval_files{i});
    [dq{i} dt{i}] = mean_diff_at_offset(eval_files{i});
    [cq{i} ct{i} offsets] = compound_error_at_offset(eval_files{i}, num_frames);
end
%true transformation should be the same for all files
[true_dq true_dt] = mean_true_diff_at_offset(eval_files{1});

frameskip = offsets - 1; %frameskip 0 means use offset 1 during registration

%Plot all the data
figure;
hold all;
title('Rotation error.');
xlabel('Frame offset');
ylabel('Mean rotation error (radian)');
for i = 1:num_methods
    plot(eq{i});
end
legend(methods, 'Location', 'SouthEast');

figure;
hold all;
title('Translation error.');
xlabel('Frame offset');
ylabel('Mean translation error (meters)');
for i = 1:num_methods
    plot(et{i});
end
legend(methods, 'Location', 'SouthEast');

figure;
hold all;
title('Magnitude of estimated rotation.');
xlabel('Frame offset');
ylabel('Mean rotation magnitude (radian)');
for i = 1:num_methods
    plot(dq{i});
end
plot(true_dq);
legend([methods {'True transformation'}], 'Location', 'SouthEast');

figure;
hold all;
title('Magnitude of estimated translation.');
xlabel('Frame offset');
ylabel('Mean translation magnitude (meters)');
for i = 1:num_methods
    plot(dt{i});
end
plot(true_dt);
legend([methods {'True transformation'}], 'Location', 'SouthEast');

figure;
hold all;
title(['Accumulated rotation error over ' int2str(num_frames) ' frames.']);
xlabel('Frameskip during registration');
ylabel('Rotation error (radian)');
for i = 1:num_methods
    plot(frameskip, cq{i});
end
legend(methods, 'Location', 'NorthEast');

figure;
hold all;
title(['Accumulated translation error over ' int2str(num_frames) ' frames.']);
xlabel('Frameskip during registration');
ylabel('Translation error (meters)');
for i = 1:num_methods
    plot(frameskip, ct{i});
end
legend(methods, 'Location', 'NorthEast');




