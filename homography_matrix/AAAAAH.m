close all
clear all

img = imread('AAAAAH.jpg');

coords = [162,302
    228, 306
    225, 356
    159, 349];

target_coords = [0,0
    200, 0
    200, 100
    0, 100];

newImage = homographic_transform2D(img,coords,target_coords);

imshow(img)
figure()
imshow(newImage)