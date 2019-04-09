close all
clear all

img = imread('street.jpg');

coords = [154, 148
    380, 235
    381, 263
    151, 202];

target_coords = [0,0
    800, 0
    800, 100
    0, 100];

newImage = homographic_transform2D(img,coords,target_coords);

imshow(img)
figure()
imshow(newImage)