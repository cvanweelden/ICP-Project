function dq = angular_distance(q1, q2)
%ANGULAR_DISTANCE compute angular distance in radian between two matrices
%of quaternions

dq = 2 * acos(dot(quatnormalize(q1), quatnormalize(q2),2));
dq = min(dq, 2*pi-dq);

end