function [ xy1_homo, xy2_homo, is_valid ] = ransac( xy1_homo, xy2_homo,...
    seed_size, thresh, trial_num, best_thresh )
% RANSAC
%--------------------------------------------------------------------------
pt_num = size(xy1_homo, 2);
best_num = 0;
is_valid = 1;
inlier_num = zeros(trial_num,1);
inlier_ind_all = cell(trial_num,1);

count = 0;
while (best_num < best_thresh) && (count <10)
    for m = 1:trial_num
        seed_ind = randsample(pt_num, seed_size);
        % Estimating homography
        H = estimateHomography( xy1_homo(:,seed_ind),xy2_homo(:,seed_ind));
        xy1_proj = H * xy1_homo;
        ssd = sum((xy1_proj - xy2_homo).^2, 1);
        inlier_ind_all{m,1} = find(ssd < thresh);
        inlier_num(m) = length(inlier_ind_all{m,1});
        
    end
    [best_num,best_ind] = max(inlier_num);
    fprintf('Max inlier percentage = %.0f %%\n',(best_num/pt_num)*100);
    thresh = thresh + 100;
    count = count + 1;
end
inlier_ind = inlier_ind_all{best_ind};
xy1_homo = xy1_homo(:,inlier_ind);
xy2_homo = xy2_homo(:,inlier_ind);
if best_num < best_thresh
    is_valid = 0;
end
end

