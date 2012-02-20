function results_object = parse_eval_file(eval_file)

% Read the output file
results = importdata(eval_file, ' ');

offset = results(:, 1);
model_t = results(:, 2:4);
model_q = results(:, [8 5:7]);
frame_t = results(:, 9:11);
frame_q = results(:, [15 12:14]);
transformation_t = results(:, 16:18);
transformation_q = results(:, [22 19:21]);

% Change from mocap to kinect coordinate system
model_t(:,2) = -model_t(:,2);
model_t(:,3) = -model_t(:,3);
frame_t(:,2) = -frame_t(:,2);
frame_t(:,3) = -frame_t(:,3);

% Compute ground truth transformations
true_transformation_t = zeros(size(transformation_t));
true_transformation_q = zeros(size(transformation_q));
for i = 1:size(results,1);
    true_transformation_q(i,:) = quatmultiply(quatinv(model_q(i,:)), frame_q(i,:));
    true_transformation_t(i,:) = frame_t(i,:) - quatrotate(true_transformation_q(i,:), model_t(i,:));
end

% Calculate true differences
diff_q = angular_distance(frame_q, model_q);
diff_t = sqrt(sum((frame_t - model_t).^2,2));


% Calculate errors
error_q = angular_distance(transformation_q, true_transformation_q);
error_t = sqrt(sum((true_transformation_t - transformation_t).^2,2));

% Put everything in the return object
results_object.offset = offset;
results_object.model_t = model_t;
results_object.model_q = model_q;
results_object.frame_t = frame_t;
results_object.frame_q = frame_q;
results_object.transformation_t = transformation_t;
results_object.transformation_q = transformation_q;
results_object.true_transformation_t = true_transformation_t;
results_object.true_transformation_q = true_transformation_q;
results_object.diff_q = diff_q;
results_object.diff_t = diff_t;
results_object.error_q = error_q;
results_object.error_t = error_t;

end
