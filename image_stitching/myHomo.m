function homo = myHomo(points1,points2)

%  points1 = [1,2,43,5,5,6,4,6,3,5,6,3,5
%      11,22,4,52,54,61,41,16,23,53,64,32,57];
%  points2 = points1;
% points2(1,:) = points2(1,:)+3;
% points2(2,:) = points2(2,:)+4;

points_homo = [points1;ones(1,size(points1,2))];
newPoints_homo = [points2;ones(1,size(points2,2))];

% newPoints_homo = T*points_homo
% b/A, Solve systems of linear equations xA = B for x, solved as a least-squares solution
homo = newPoints_homo/points_homo;
end