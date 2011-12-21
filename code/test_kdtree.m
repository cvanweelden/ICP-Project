%% Load data
config
load([DATA_PATH '/2frames.mat']);

model_points = im2pixels(D1);


%% Using the C implementation


numpoints = 1:10000:300000;
times = zeros(1,numel(numpoints));
for i=1:numel(numpoints)
  tic
  tree = kdtree_build(model_points(1:numpoints(i),:));
  times(i) = toc;
  kdtree_delete(tree);
  plot(numpoints, times);
  drawnow
end


%% Using the pure matlab implementation

numpoints = 1:100:300000;
times = zeros(1,numel(numpoints));
for i=1:numel(numpoints)
  tic
  tree = kd_buildtree(model_points(1:numpoints(i),:), false);
  times(i) = toc;
  plot(numpoints, times);
  drawnow
end
