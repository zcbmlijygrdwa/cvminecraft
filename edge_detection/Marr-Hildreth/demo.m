close all;
clear all;


imgs = my_edge_detector('data/lena.jpg')

%show results
%visulization(img,img_out_3,img_out_6,img_out_12,img_out_24,img_out_48);
for i = 1:length(imgs)
    figure(),imshow(imgs{i})
end


