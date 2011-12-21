function [ q_r q_t ] = get_transformation( P, X )
%GET_TRANSFORMATION compute registration vector of data P to model X
%   P and X are 3xN matrixes for N points in 3 dimensions, i.e. each column
%   corresponds to a single point. See Besl & McKay (1992) section III.C.

mu_p = sum(P,2)/size(P,2);
mu_x = sum(X,2)/size(X,2);

Sigma = (P*X')/size(P,2) - mu_p*mu_x';

A = (Sigma - Sigma');
Delta = [A(2,3) A(3,1) A(1,2)]';

tr = trace(Sigma);
Q = [tr Delta'; Delta (Sigma + Sigma' - tr*eye(3))];

[V,D] = eig(Q);
[maxValue,index] = max(diag(D));
q_r = V(:,index);

q_t = mu_x - quad2rot(q_r) * mu_p;

end

