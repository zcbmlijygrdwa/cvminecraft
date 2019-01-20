
close all;
clear all;

%img = imread('data/lena.jpg');
%img = imread('data/lena3.bmp');
%img = imread('data/ucsb.bmp');
%img = imread('data/up.bmp');
img = imread('data/up.jpg');

imgs = my_edge_detector('data/up.jpg')

%show results
%visulization(img,img_out_3,img_out_6,img_out_12,img_out_24,img_out_48);
for i = 1:length(imgs)
    figure(),imshow(imgs{i})
    imwrite(imgs{i},['output/MyImage_',num2str(i-1),'.bmp']);
end

% Output five binary edge maps at five different scales

