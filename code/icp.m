function [ P mse ] = icp( P, X, mse_threshold )
%ICP Iterated Closest Point allignment of data P to model X
%   P and X are 3xN matrixes for N points in 3 dimensions, i.e. each row
%   corresponds to a single point. MSE_THRESHOLD is the desired precision
%   in mean squared errror.

%Octree for model data X
tree = kdtree_build(X');
NB = zeros(size(P,2),1);

%Run the ICP loop untill MSE is small enough
mse = inf;
for j = 1:250

    %NB(i) = index for nearest neighbor of P(i) in X
    for i = 1:size(P,2)
        NB(i) = kdtree_k_nearest_neighbors(tree, P(:,i)', 1);
    end
    X_NB = X(:,NB); % Reorder X
    
    %Compute and apply the transformation
    [q_r, q_t] = get_transformation(P, X_NB);
    P = quat2rot(q_r) * P + repmat(q_t,1,size(P,2));
    mse = mean(sum((P-X_NB).^2,2));
    
    if mse < mse_threshold
      break
    end
    fprintf('MSE: %g > %g\n', mse, mse_threshold);
    
end

kdtree_delete(tree)

end

