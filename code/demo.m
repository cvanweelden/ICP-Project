bla = load([DATA_PATH 'DandI.mat']);

D = bla.D;

figure;

D1 = D{1}(1:25:end, 1:25:end, :);
X1 = D1(:,:,1);
Y1 = D1(:,:,2);
Z1 = D1(:,:,3);
D1 = [X1(:)'; Y1(:)'; Z1(:)'];

D1(:,D1(1,:)==0 & D1(2,:)==0 & D1(3,:)==0) = [];
hold on;
scatter3(D1(1,:), D1(2,:), D1(3,:), 1, 'b')

for i = 2:10

    D2 = D{i}(1:25:end, 1:25:end, :);
    X2 = D2(:,:,1);
    Y2 = D2(:,:,2);
    Z2 = D2(:,:,3);
    D2 = [X2(:)'; Y2(:)'; Z2(:)'];
    D2(:,D2(1,:)==0 & D2(2,:)==0 & D2(3,:)==0) = [];

    [D2_alligned mse] = icp(D2, D1, 0.5);
    scatter3(D2_alligned(1,:), D2_alligned(2,:), D2_alligned(3,:), 1, 'b');
    D1 = D2_alligned;
end

