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

img1 = single(img1);
img2 = single(img2);



boxPoints = detectSURFFeatures(img1);
 
% [f1,d1] = vl_sift(img1, 'Levels', levels, 'WindowSize',win_size,...
%     'EdgeThresh', edge_thresh, 'Magnif', 10);
% [f2,d2] = vl_sift(img2, 'Levels', levels, 'WindowSize',win_size,...
%     'EdgeThresh', edge_thresh, 'Magnif', 10);
% 
% [matches, scores] = vl_ubcmatch(d1, d2) ;
% 


[asc_score, inc_ind] = sort(scores); % Sort score in ascending order

if is_ret_xy
    matches = matches(:,inc_ind); % Lower index imply better match
    xy1_all = f1(1:2,matches(1,:)); % Lower index imply better match
    xy2_all = f2(1:2,matches(2,:)); % Lower index imply better match
else
    xy1_all = [];
    xy2_all = [];
end

output_score_len = min(50, length(asc_score));
avg_score = mean(asc_score(1:output_score_len)); % Average score (1-50)
end