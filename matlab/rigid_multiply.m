function [ qt2 ] = rigid_multiply(dqt, qt1)

% dq = quatnormalize(dqt(1:4));
% q = quatnormalize(qt(1:4));


t2 = quatrotate(dqt(1:4), qt1(5:7)) + dqt(5:7);
q2 = quatmultiply(qt1(1:4), dqt(1:4));

qt2 = [q2 t2];
            
% qt(5:7) = dqt(5:7) + quatrotate(dqt(1:4), qt(5:7));
% qt(1:4) = quatmultiply(qt(1:4), dqt(1:4));


end