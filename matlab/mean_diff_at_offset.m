function [mean_diff_t mean_diff_q] = mean_diff_at_offset(eval_file)
%MEAN_ERROR_AT_OFFSET Compute the mean estimated difference (angular 
%distance (rad) and euclidean distance) estimated at each offset for a file
%generated using the evaluation script (see evaluation.cpp).

results = parse_eval_file(eval_file);

max_offset = max(results.offset);

mean_diff_q = zeros(1,max_offset);
mean_diff_t = zeros(1,max_offset);
for i = 1:max_offset
    transformation_q_at_i = results.transformation_q(results.offset == i);
    diff_q_at_i = 2 * acos(transformation_q_at_i(:,1)); %angular distance with [1 0 0 0], see angular distance function
    diff_q_at_i = min(diff_q_at_i, 2*pi-diff_q_at_i);
    mean_diff_q(i) = mean(diff_q_at_i);
    
    transformation_t_at_i = results.transformation_t(results.offset == i);
    diff_t_at_i = sqrt(sum(transformation_t_at_i.^2,2));
    mean_diff_t(i) = mean(diff_t_at_i);
end
    
end