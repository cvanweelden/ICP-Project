function [A mse_profile] = gicp( model, frame, varargin)
% Generalized ICP
%   P and X are point clouds.

p = inputParser;
p.addParamValue('MaxIterations', 50, @(x)isnumeric(x));
p.addParamValue('MSEThreshold',  1e-5, @(x)isnumeric(x));
p.parse(varargin{:});

maxiter = p.Results.MaxIterations;
msethresh = p.Results.MSEThreshold;

P = model.xyz;
X = frame.xyz;

%Octree for model data X
tree = kdtree_build(X');
NB = zeros(size(P,2),1);

%The final transformation:
A = eye(4);

%Run the ICP loop until difference in MSE is < msethresh
mse_profile = zeros(maxiter,1);

mse = inf;
for j = 1:maxiter
    
    %NB(i) = index for nearest neighbor of P(i) in X
    for i = 1:size(P,2)
        NB(i) = kdtree_k_nearest_neighbors(tree, P(:,i)', 1);
    end
    X_NB = X(:,NB); % Reorder X
    
    %Compute and apply the transformation
    %TODO this should be different for G-ICP
    [q_r, q_t] = get_transformation(P, X_NB);
    A = [quat2rot(q_r) q_t; 0 0 0 1] * A;
    P = [quat2rot(q_r) q_t; 0 0 0 1] * [P; ones(1, size(P,2))];
    P = P(1:3, :);
    mse_n = mean(sum((P-X_NB).^2,2));
    
    if mse - mse_n < msethresh
      break
    end
    mse = mse_n;
    mse_profile(j) = mse;
end
mse_profile(j+1:end) = [];

kdtree_delete(tree)

end

