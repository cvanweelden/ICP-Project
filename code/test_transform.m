q = angle2quat( 0.8, 0.2, 0.3 );
t = [0.13; 0.7; 0.42];
X = rand(3,50);
P = quat2rot(q) * X + repmat(t,1,size(X,2));
[q_r q_t] = get_transformation(P, X);
P2 = quat2rot(q_r) * P + repmat(q_t,1,size(P,2));
mse = mean(sum((P2-X).^2,2))