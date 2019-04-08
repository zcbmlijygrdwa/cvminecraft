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

 T = [0,2,0];
 
 trans = [R,T';0,0,0,1]
 
 camera_center2 = (trans*[camera_center1;1]);
 camera_center2 = camera_center2(1:3,:);
 
 
 
 
%epipolar point in image 1
temp = trans*[camera_center2;1];
temp = temp(1:3,:);
e_1 = K*temp;
e_1 = e_1/e_1(3);
e_1 = e_1(1:2,:)
 
 
%epipolar point in image 2
temp = inv(trans)*[camera_center1;1];
temp = temp(1:3,:);
e_2 = K*temp;
e_2 = e_2/e_2(3);
e_2 = e_2(1:2,:)
 
figure(1)
scatter(e_1(1,:),e_1(2,:));
xlim([0,image_width]);
ylim([0,image_height]);

figure(2)
scatter(e_2(1,:),e_2(2,:));
xlim([0,image_width]);
ylim([0,image_height]);


 
 