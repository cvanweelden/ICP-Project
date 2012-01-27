function [ qt ] = rigid_multiply(dqt, qt)

dq = quatnormalize(dqt(1:4));
q = quatnormalize(qt(1:4));

qt(1:4) = quatmultiply(dq, q);
qt(5:7) = qt(5:7) + quatrotate(quatinv(q), dqt(5:7));

end