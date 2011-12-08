config
load([DATA_PATH '/2frames.mat']);

model_points = im2pixels(D1);

numpoints = 1:1000:300000;
times = zeros(1,numel(numpoints));
for i=1:numel(numpoints)
tic
tree = kdtree_build(model_points(1:numpoints(i),:));
times(i) = toc;
kdtree_delete(tree);
plot(numpoints, times);
drawnow
end

