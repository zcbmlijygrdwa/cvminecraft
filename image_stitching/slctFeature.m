function [ xy1_homo, xy2_homo ] = slctFeature( xy1_all, xy2_all,...
    slct_num, dist_thresh )
match_len = size(xy1_all, 2);
slct_num = min(slct_num, size(xy1_all, 2));

xy1 = [xy1_all(:,1), zeros(2, slct_num-1)];
xy2 = [xy2_all(:,1), zeros(2, slct_num-1)];

count = 1;
i=2;
while (count < slct_num)&&(i <= match_len)
    dist1 = norm(xy1_all(:,i) - xy1(:,count));
    dist2 = norm(xy2_all(:,i) - xy2(:,count));
    if (dist1>dist_thresh)&&(dist2>dist_thresh)
        count = count + 1;
        xy1(:,count) = xy1_all(:,i);
        xy2(:,count) = xy2_all(:,i);
    end
    i = i + 1;
end

xy1_homo = [ xy1; ones( 1, size(xy1, 2))];
xy2_homo = [ xy2; ones( 1, size(xy2, 2))];
end