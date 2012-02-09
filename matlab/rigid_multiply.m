function [ qt ] = rigid_multiply(dqt, qt)

% dq = quatnormalize(dqt(1:4));
% q = quatnormalize(qt(1:4));


qt(5:7) = qt(5:7) + quatrotate(qt(1:4), dqt(5:7));
qt(1:4) = quatmultiply(qt(1:4), dqt(1:4));


end