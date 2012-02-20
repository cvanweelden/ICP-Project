function [error_q error_t] = compound_error_at_offset(eval_file, num_frames)
%COMPOUND_ERROR_AT_OFFSET Compute the error (angular distance (rad) and
%euclidean distance) at each possible offset between registered frames
%compared between the estimated and true pose after NUM_FRAMES frames for a
%file generated using the evaluation script (see evaluation.cpp).

results = parse_eval_file(eval_file);

max_offset = min(num_frames,max(results.offset));

true_q = results.
true_t

error_q = zeros(1,max_offset);
error_t = zeros(1,max_offset);
for i = 1:max_offset
    


end