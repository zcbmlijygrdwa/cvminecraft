%% CS281B Homework Assignment 3: Panorama Stitching
% Author: Yitian Shao
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
%--------------------------------------------------------------------------
% To stitch images imported in random sequence
% Determine stitch sequence------------------------------------------------
pair_score = zeros(num_of_img); %'[Stitched]*[Not]'
for i = 1:num_of_img-1
    for j = i+1:num_of_img
        [~,~, scores] = featureExtraction( images{i}, images{j}, 0,...
            levels, win_size, edge_thresh );
        fprintf('Match score of image %d and %d: %.1f\n', i, j, scores);
        pair_score(i,j)= scores;
    end
end
pair_score = pair_score + pair_score'; % Flip upper triangle
img_I = 1:num_of_img;
% Find the optimal pairs (and sequence) to stitch images
pair_info = zeros(num_of_img-1,2); % '[i, j]'
row_info = [1, zeros(1,num_of_img-1)]; % 'Stitched=1'
col_info = bsxfun(@xor, row_info, ones(1,num_of_img));
for k = 1:num_of_img-1
    row_ind = img_I(logical(row_info));
    col_ind = img_I(logical(col_info));
    stitch_pool = pair_score(row_ind, col_ind);
    [~, score_ind] = min(stitch_pool(:));
    [pool_i, pool_j] = ind2sub(size(stitch_pool),score_ind);
    pair_info(k,1) = row_ind(pool_i);
    pair_info(k,2) = col_ind(pool_j);
    row_info(pair_info(k,:)) = 1;
    col_info = bsxfun(@xor, row_info, ones(1,num_of_img));
end
fprintf('Time to pair images = %.2f sec\n', toc);

% Using histogram to determine stitch sequence
[stitch_freq,~] = histcounts(pair_info); % Stitch frequency of each images
[~,node_ind] = max(stitch_freq); % Determine the root

% Construct a tree sturct to determine the stitch order 
level = 1;
level_info = zeros(num_of_img,3);
level_info(node_ind, 1) = level; % Base level = 1
level_info(node_ind, 2) = node_ind; % Parent index
level_info(node_ind, 3) = 1; % Search index
H_info = cell(num_of_img, num_of_img-1); % Store tree infomation

% Recursive function to contruct the tree determing the sequence
[level_info,H_info]=stitchSeq(level,1,level_info,pair_info,num_of_img,...
    H_info);

%% ========================================================================
% Begin stitching images
deep_len = max(level_info(:,1))-1; % Depth of the tree structure
H_pack = cell(num_of_img, deep_len); % Storage of transformation matrices

for n = 1:deep_len
    for m = 1:num_of_img
        if ~isempty(H_info{m,n}) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
i = H_info{m,n}(1);
j = H_info{m,n}(2);
fprintf('Projection of image [%d] to [%d] \n', i,j);
% Feature extraction
[xy1, xy2,~] = featureExtraction(images{i}, images{j}, 1,...
    levels, win_size, edge_thresh);
% Feature selection
[ xy1_slct, xy2_slct ] = slctFeature( xy1, xy2, slct_num, dist_thresh );
% plotFeature( images{i}, images{j}, xy1_slct, xy2_slct ); % For debug only

% RANSAC===================================================================
tic
[ xy1_homo, xy2_homo, is_valid ] = ransac( xy1_slct, xy2_slct,...
    seed_size, thresh, trial_num, best_thresh );
% plotFeature( images{i}, images{j}, xy1_homo, xy2_homo ); % For debug only
fprintf('Time of RANSAC = %.2f sec\n - With %d feature points selected\n',...
    toc, size(xy1_homo, 2));
%--------------------------------------------------------------------------
if is_valid
    H_pack{m,n} = estimateHomography( xy1_homo, xy2_homo);
else
    % Try redo feature extraction
    [xy1, xy2,~] = featureExtraction(images{i}, images{j}, 1,...
        levels, win_size, edge_thresh);
    [ xy1_slct, xy2_slct ] = slctFeature(xy1, xy2, slct_num, dist_thresh);
    [ xy1_homo, xy2_homo, is_valid ] = ransac( xy1_slct, xy2_slct,...
        seed_size, thresh + 1000, trial_num, best_thresh );
% plotFeature( images{i}, images{j}, xy1_homo, xy2_homo ); % For debug only
    if is_valid
        try
            H_pack{m,n} = estimateHomography( xy1_homo, xy2_homo);
        catch
            H_pack{m,n} = eye(3);
            fprintf('Failed to stitch image %d and %d\n', i, j);
        end
    else
        H_pack{m,n} = eye(3);
        fprintf('Failed to stitch image %d and %d\n', i, j);
    end
end
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
% -------------------------------------------------------------------------
% Compute transformation matrices with respect to overall reference
for n = deep_len:-1:1
    for m = 1:num_of_img
        if isempty(H_info{m,n}) && (level_info(m,1) > n)
            H_info{m,n} = H_info{H_info{m,n+1}(2),n};
            H_pack{m,n} = H_pack{H_info{m,n+1}(2),n}*H_pack{m,n+1};
        end
    end
end
% -------------------------------------------------------------------------
%% Stitch images together==================================================
tic;
ext_off_y = 0;
ext_off_x = 0;
for m = 1:num_of_img
    if ~isempty(H_pack{m,1})
        i = H_info{m,level_info(m,1)-1}(1);
        j = H_info{m,1}(2);
        [stitch_img, off_y, off_x] = stitchImage2(images,...
            i, j, H_pack{m,1}, ext_off_y, ext_off_x, 1);
        ext_off_y = ext_off_y + off_y;
        ext_off_x = ext_off_x + off_x;
        images{j} = stitch_img;
    end
end
fprintf('Time to stitch image %d and %d = %.2f sec\n', i, j, toc);     

%--------------------------------------------------------------------------  
% Amend image==============================================================
if is_interpolated
%% Interpolate the iamge afterward: Run this section
    tic
    amend_img = amend( stitch_img );
    figure('Position', [1400,150,600,400])
    imshow(amend_img)
    title('Amended stitched image')
    fprintf('Time to amend image = %.2f sec\n', toc);
end
end