function points2 = homoTransform (homo,points1)
%homography transform from points1 to points2
points1_homo = [points1;ones(1,size(points1,2))];
points2_homo = homo*points1_homo;
points2 = points2_homo(1:2,:);
end