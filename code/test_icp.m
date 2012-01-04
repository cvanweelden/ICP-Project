bla = load([DATA_PATH 'DandI.mat']);

D = bla.D;

figure;

D1 = D{1}(1:25:end, 1:25:end, :);
D1 = im2pixels(D1)';

D1(:,D1(1,:)==0 & D1(2,:)==0 & D1(3,:)==0) = [];
hold on;
scatter3(D1(1,:), D1(2,:), D1(3,:), 1, 'b')

for i = 2:10

    D2 = D{i}(1:25:end, 1:25:end, :);
    D2 = im2pixels(D2)';
    D2(:,D2(1,:)==0 & D2(2,:)==0 & D2(3,:)==0) = [];

    [D2_alligned mse] = icp(D2, D1, 0.5);
    scatter3(D2_alligned(1,:), D2_alligned(2,:), D2_alligned(3,:), 1, 'rx');
    D1 = D2_alligned;
end

