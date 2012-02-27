function [mean_error_q mean_error_t] = mean_error_at_offset(eval_file)
%MEAN_ERROR_AT_OFFSET Compute the mean error (angular distance (rad) and
%euclidean distance) at each offset for a file generated using the
%evaluation script (see evaluation.cpp).

results = parse_eval_file(eval_file);

max_offset = max(results.offset);

mean_error_q = zeros(1,max_offset);
mean_error_t = zeros(1,max_offset);
for i = 1:max_offset
    error_q_at_i = results.error_q(results.offset == i);
    mean_error_q(i) = mean(error_q_at_i);
    error_t_at_i = results.error_t(results.offset == i);
    mean_error_t(i) = mean(error_t_at_i);
end

end