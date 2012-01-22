function [P Q mse mse_profile num_iter] = icp( P, X, par)
%ICP Iterated Closest Point allignment of data P to model X
%   P and X are HEIGHTxWIDTHx3 point clouds. MSE_THRESHOLD is the desired
%   precision in mean squared errror. DEBUG (optional) 0 (default):no debug
%   info, 1:print MSE on each iteration.

if (nargin < 3)
    par.num_iterations = 1000;
    par.mse_threshold = 0.01;
    par.debug = 0;
end

%Octree for model data X
tree = kdtree_build(X');
NB = zeros(size(P,2),1);

%The final transformation:
Q = eye(4);

%Run the ICP loop untill MSE is small enough
mse_profile = zeros(par.num_iterations,1);
for j = 1:par.num_iterations

    %NB(i) = index for nearest neighbor of P(i) in X
    for i = 1:size(P,2)
        NB(i) = kdtree_k_nearest_neighbors(tree, P(:,i)', 1);
    end
    X_NB = X(:,NB); % Reorder X
    
    %Compute and apply the transformation
    [q_r, q_t] = get_transformation(P, X_NB);
    Q = [quat2rot(q_r) q_t; 0 0 0 1] * Q;
    P = [quat2rot(q_r) q_t; 0 0 0 1] * [P; ones(1, size(P,2))];
    P = P(1:3, :);
    mse = mean(sum((P-X_NB).^2,2));
    
    if mse < par.mse_threshold
      break
    end
    if (par.debug > 0)
        fprintf('MSE: %g > %g\n', mse, par.mse_threshold);
    end
    
    mse_profile(j) = mse;
end
mse_profile(j+1:end) = [];
num_iter = j;

kdtree_delete(tree)

end

