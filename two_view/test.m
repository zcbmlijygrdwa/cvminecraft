close all
clear all

addpath('../../matlabplugins')

image_width = 640;
image_height = 480;



fx = 100;
fy = 100;

cx = image_width/2;
cy = image_height/2;



K = [fx 0 cx
    0 fy cy
    0 0 1];



points_3D = zeros(3,8);


for i=1:size(points_3D,2)
    points_3D(:,i) = [randi(10),randi(10),randi(10)]';
end



camera_center1 = [0,0,0]';


R = rotmat3d(0.9,0,0);

t = [0,2,0];

T = [R,t';0,0,0,1]

camera_center2 = (T*[camera_center1;1]);
camera_center2 = camera_center2(1:3,:);





%projection


points_3D_homo = [points_3D;ones(1,size(points_3D,2))]
temp = eye(4)*points_3D_homo;
temp = temp(1:3,:)

points_2D_1_homo = K*temp;
for i = 1:size(points_3D,2)
    points_2D_1_homo(:,i) = points_2D_1_homo(:,i)/points_2D_1_homo(3,i);
end
points_2D_1 = points_2D_1_homo(1:2,:)


temp = T*points_3D_homo;
temp = temp(1:3,:)

points_2D_2_homo = K*temp;
for i = 1:size(points_3D,2)
    points_2D_2_homo(:,i) = points_2D_2_homo(:,i)/points_2D_2_homo(3,i);
end
points_2D_2 = points_2D_2_homo(1:2,:)




%epipolar point in image 1
temp = T*[camera_center2;1];
temp = temp(1:3,:);
e_1 = K*temp;
e_1 = e_1/e_1(3)


%epipolar point in image 2
temp = inv(T)*[camera_center1;1];
temp = temp(1:3,:);
e_2 = K*temp;
e_2 = e_2/e_2(3)

figure(1)
scatter(e_1(1,:),e_1(2,:));
xlim([0,image_width]);
ylim([0,image_height]);

figure(2)
scatter(e_2(1,:),e_2(2,:));
xlim([0,image_width]);
ylim([0,image_height]);



P = K*[eye(3),[0;0;0]];

P_prime = K*[R,t'];

P_plus = [inv(K);[0,0,0]];


F = skew_sym_mat(e_1)*P_prime*P_plus

rank_F = rank(F)


%get Fundamental matrix from 8 points


%contruct A
A = zeros(size(points_3D,2),9);

%http://www.cse.psu.edu/~rtc12/CSE486/lecture20_6pp.pdf
for i=1:size(points_3D,2)
    A(i,:) = [points_2D_1(1,i)*points_2D_2(1,i), points_2D_1(1,i)*points_2D_2(2,i), points_2D_1(1,i), points_2D_1(2,i)*points_2D_2(1,i), points_2D_1(2,i)*points_2D_2(2,i), points_2D_1(2,i), points_2D_2(1,i), points_2D_2(2,i), 1];
end


[U,S,V] = svd(A);

F_8_points = reshape(V(:,8),3,3);

%add constrain that F has rank of 2

[U,S,V] = svd(F_8_points);

S(3,3) = 0;

F_8_points = U*S*V'

rank_F_8_points = rank(F_8_points)
