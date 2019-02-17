function globalImage = projection(homo,globalImage,sourceImage)
homo_inv = inv(homo);
for i = 1:size(sourceImage,1)
    i
    for j = 1:size(sourceImage,2)
        x_img2 = j;
        y_img2 = i;
        locInImg2 = homo_inv*[x_img2 y_img2 1]';
        locInImg2 = int32(locInImg2);
         i_img1_new = locInImg2(2);
         j_img1_new = locInImg2(1);
         
         i_img1_new = i_img1_new+size(globalImage,1)/2;
         j_img1_new = j_img1_new+size(globalImage,2)/2;
         
        if(i_img1_new>=1&&i_img1_new<=size(globalImage,1)...
                &&j_img1_new>=1&&j_img1_new<=size(globalImage,2))
        globalImage(i_img1_new,j_img1_new) = sourceImage(i,j);
        end
    end
end
end