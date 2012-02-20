function results_object = parse_eval_file(eval_file)

% Read the output file
results = importdata(eval_file, ' ');

model_index = results(:, 1);
frame_index = results(:, 2);
offset = frame_index - model_index;
model_t = results(:, 3:5);
model_q = results(:, [9 6:8]);
frame_t = results(:, 10:12);
frame_q = results(:, [16 13:15]);
transformation_t = results(:, 17:19);
transformation_q = results(:, [23 20:22]);

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
results_object.model_index = model_index;
results_object.frame_index = frame_index;
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
