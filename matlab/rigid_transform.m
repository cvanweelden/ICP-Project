function [ At ] = rigid_transform(qt, A)

q = quatnormalize(qt(1:4));
T = [quat2dcm(q)' qt(5:7)'; 0 0 0 1];
At = T * [A; ones(1,size(A,2))];
At = At(1:3, :);
end