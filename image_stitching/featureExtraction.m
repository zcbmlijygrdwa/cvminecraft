function [ xy1_all, xy2_all, avg_score ] = featureExtraction( img1, img2,...
    is_ret_xy, levels, win_size, edge_thresh)
% Updated on 05/05/2016
%--------------------------------------------------------------------------
if size(img1, 3) > 1
    img1 = rgb2gray(img1);
end
if size(img2, 3) > 1
    img2 = rgb2gray(img2);
end

% img1 = single(img1);
% img2 = single(img2);



points1  = detectSURFFeatures(img1);
points2  = detectSURFFeatures(img2);

%Extract the features descriptors

[f1,vpts1] = extractFeatures(img1,points1);
[f2,vpts2] = extractFeatures(img2,points2);

% Retrieve the locations of matched points.

indexPairs = matchFeatures(f1,f2,'Unique',true,'MaxRatio',0.4) ;
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));
 figure; showMatchedFeatures(img1,img2,matchedPoints1,matchedPoints2);
legend('matched points 1','matched points 2');

homo = myRansac(indexPairs,matchedPoints1,matchedPoints2,100);

% homo_inv = inv(homo);
img1_new = zeros(3000,8000);

for i = 1:size(img1,1)
    i
    for j = 1:size(img1,2)
        new_i = size(img1_new,1)/2+i;
        new_j = size(img1_new,2)/2+j;
        
        img1_new(new_i,new_j) = img1(i,j);
    end
end

img1_new = projection(homo,img1_new,img2);

% for i = 1:size(img2,1)
%     i
%     for j = 1:size(img2,2)
%         x_img2 = j;
%         y_img2 = i;
%         locInImg2 = homo_inv*[x_img2 y_img2 1]';
%         locInImg2 = int32(locInImg2);
%          i_img1_new = locInImg2(2);
%          j_img1_new = locInImg2(1);
%          
%          i_img1_new = i_img1_new+size(img1_new,1)/2;
%          j_img1_new = j_img1_new+size(img1_new,2)/2;
%          
%         if(i_img1_new>=1&&i_img1_new<=size(img1_new,1)...
%                 &&j_img1_new>=1&&j_img1_new<=size(img1_new,2))
%         img1_new(i_img1_new,j_img1_new) = img2(i,j);
%         end
%     end
% end


figure,imshow(img1_new,[])

a = 1;
end