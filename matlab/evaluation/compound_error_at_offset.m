function [mean_error_q mean_error_t offsets] = compound_error_at_offset(eval_file, num_frames)
%COMPOUND_ERROR_AT_OFFSET Compute the error (angular distance (rad) and
%euclidean distance) at each possible offset between registered frames
%compared between the estimated and true pose after NUM_FRAMES frames for a
%file generated using the evaluation script (see evaluation.cpp).
%FIXME: bad explanation :/

results = parse_eval_file(eval_file);

max_offset = min(num_frames,max(results.offset));
max_model_index = max(results.model_index);
max_frame_index = max(results.frame_index);

%find all integer dividers of num_frames
offsets = 1:max_offset;
offsets = offsets(mod(num_frames,offsets)==0);

error_q = zeros(1, length(offsets));
error_t = zeros(1, length(offsets));
z = zeros(1, length(offsets));
for start_index = 0:min(max_model_index, max_frame_index - num_frames )
    for stepsize = offsets
        num_steps = num_frames/stepsize;
        if (start_index <= max_model_index - (num_steps-1)*stepsize)
        
            %Calculate the error for one stepsize
            estimated_qt = [1 0 0 0 0 0 0];
            true_qt = [1 0 0 0 0 0 0];
            for i = 0:num_steps-1
                model_index = start_index + i*stepsize;
                frame_index = model_index + stepsize;
                index = results.model_index == model_index & results.frame_index == frame_index;
                estimated_step_qt = [results.transformation_q(index,:) results.transformation_t(index,:)];
                true_step_qt = [results.true_transformation_q(index,:) results.true_transformation_t(index,:)];
                estimated_qt = rigid_multiply(estimated_step_qt, estimated_qt);
                true_qt = rigid_multiply(true_step_qt, true_qt);
            end
            e_index = find(offsets==stepsize);
            error_q(e_index) = error_q(e_index) + angular_distance(true_qt(1:4), estimated_qt(1:4));
            error_t(e_index) = error_t(e_index) + sqrt(sum((true_qt(5:7) - estimated_qt(5:7)).^2,2));
            z(e_index) = z(e_index) + 1;
        end
    end
end

mean_error_q = error_q ./ z;
mean_error_t = error_t ./ z;

end