%% CS281B Homework Assignment 3: Panorama Stitching
% Author: Zhenyu Yang
%--------------------------------------------------------------------------
% Program description:
% 1. This program can stitch images with same name in designated path to
% produce panaroma image.
% 2. This program can take images in RANDOM SEQUENCES
% 3. The stitched panorama will be interpolated to reduce unwanted
% artifacts caused by stitching.
%==========================================================================
% The feature extraction part of this program is implemented using the
% VLFeat open source library - http://www.vlfeat.org/.
% Functions used: 'vl_sift' and 'vl_ubcmatch'. Everything else is
% self-written.
%
% VLFeat setup instruction - http://www.vlfeat.org/install-matlab.html
%==========================================================================
% Main script==============================================================
close all
clear all
clc

% Setup VLFeat library=====================================================
%run('vlfeat-0.9.20/toolbox/vl_setup');
%==========================================================================
% Set path (Please change the path accordingly if testing it on other images)
Data_path = 'GrandCanyon1/' % Stitching "GrandCanyon1"
Image_separator = '_';
%==========================================================================
% Tuning parameters========================================================
%==========================================================================
% Feature extraction
levels = 4; % Set the number of levels per octave of the DoG scale space
win_size = 4; % Set the variance of the Gaussian window
edge_thresh = 10; % Set the non-edge selection threshold

slct_num = 60; % Number of selected features from SIFT
dist_thresh = 20; % Minimum 2-norm distance between selected feature
% RANSAC-------------------------------------------------------------------
seed_size = 4; % Number of random selected features in RANSAC
thresh = 400; % Initial SSD threshold
trial_num = 50; % Number of trials
best_thresh = 10; % Minimum inlier number
% Interpolation------------------------------------------------------------
is_interpolated = 1; % Whether perform interpolation on final result
%==========================================================================
%% ========================================================================
% Import images from designated path (folder)
% Images are automatically classified based on their name
%--------------------------------------------------------
data_path=dir(Data_path);
file_num = length(data_path)-2;
img_info=cell(file_num,2); % {name, image index}
images=cell(file_num,1);

img_class_num = 1;
a_name = data_path(img_class_num+2).name;
a_name = a_name(1:find(a_name==Image_separator)-1);
if isempty(a_name)
    a_name = 'UnknownImage';
end
img_info{img_class_num,1} = a_name;
img_info{img_class_num,2} = 1;

for i = 2:file_num
    a_name = data_path(i+2).name; % Classify images by their names
    a_name = a_name(1:find(a_name==Image_separator)-1);
    if isempty(a_name)
        a_name = 'UnknownImage'; % When image name is unreadable
    end
    class_ind = 0;
    for j = 1:img_class_num
        if strcmpi(a_name,img_info{j,1})
            class_ind = j;
        end
    end
    if class_ind > 0 % Existing class of image
        img_info{j,2} = [img_info{j,2}, i];
    elseif class_ind == 0 % New class of image is found
        img_class_num = img_class_num + 1;
        img_info{img_class_num,1} = a_name;
        img_info{img_class_num,2} = i;
    end
end
for j = 1:img_class_num
    fprintf('Image (Type %d): [%s] contains %d images\n',j,...
        img_info{j,1}, length(img_info{j,2}));
end

%==========================================================================
for type_j = 1:img_class_num % Stitch images (same class) in target path
    %--------------------------------------------------
    disp('--------------------------------------------');
    tic
    num_of_img = length(img_info{type_j,2});
    images = cell(num_of_img, 1);
    for import_k = 1:num_of_img
        file_name=[Data_path, data_path(img_info{type_j,2}(import_k)+2).name];
        images{import_k} = imread(file_name);
    end
    
    
    %get feature for each image
    features = [];
    dataSet = cell(num_of_img,1);
    for i = 1:num_of_img
        dataSet{i}.img_c = images{i};
        dataSet{i}.img = autoGray(images{i});
        
        tempFeatures.points  = detectSURFFeatures(dataSet{i}.img);
        [des,vpts] = extractFeatures(dataSet{i}.img,tempFeatures.points);
        tempFeatures.des = des;
        
        dataSet{i}.features = tempFeatures;
    end
    
    
    img1_new = zeros(3000,8000);
    homo_global = eye(3);
    
    %draw the first image
    for i = 1:size(images{1},1)
        i
        for j = 1:size(images{1},2)
            new_i = size(img1_new,1)/2+i;
            new_j = size(img1_new,2)/2+j;
            
            img1_new(new_i,new_j) = images{1}(i,j);
        end
    end
    
    

    for i = 1:num_of_img-1
        img1 = dataSet{i}.img;
        img2 = dataSet{i+1}.img;
        
        [indexPairs, matchedPoints1,matchedPoints2 ] = tryMatch(dataSet{i}.features, dataSet{i+1}.features);
        
        figure; showMatchedFeatures(img1,img2,matchedPoints1,matchedPoints2);
        legend('matched points 1','matched points 2');
        
        if(size(indexPairs,1)<20)
            continue;
        end
        
        homo = myRansac(indexPairs,matchedPoints1,matchedPoints2,100);
        homo_global = homo*homo_global;
        img1_new = projection(homo_global,img1_new,img2);
        
        figure,imshow(img1_new,[])
        
        a = 1;
    end
end