function [ stitch_img ] = amend( stitch_img )
% Amend shattered area
    old_img = stitch_img;
    mend_rad = 2;
    thresh = 4 * mend_rad * mend_rad;
    block_ind = -mend_rad:mend_rad;
    mask = zeros(size(stitch_img(:,:,1)));
    mask((mend_rad+1):(end-mend_rad), (mend_rad+1):(end-mend_rad)) = 1;
    mend_ind = (stitch_img(:,:,1)==0);
    if size(stitch_img, 3)==3
        mend_ind = bsxfun(@and, mend_ind, (stitch_img(:,:,2)==0));
        mend_ind = bsxfun(@and, mend_ind, (stitch_img(:,:,3)==0));
    end
    mend_ind = bsxfun(@and, mend_ind, mask);
    [mend_x, mend_y]=ind2sub(size(mend_ind), find(mend_ind > 0));
    for i = 1:length(mend_x)
        block_r = old_img(mend_x(i)+block_ind, mend_y(i)+block_ind, 1);
        block_g = old_img(mend_x(i)+block_ind, mend_y(i)+block_ind, 2);
        block_b = old_img(mend_x(i)+block_ind, mend_y(i)+block_ind, 3);
        avg_ind = bsxfun(@and, (block_r>0), (block_g>0));
        avg_ind = bsxfun(@and, avg_ind, (block_b>0));
        if numel(avg_ind) > thresh
            stitch_img(mend_x(i), mend_y(i), :) =...
                uint8([mean(block_r(avg_ind));...
                mean(block_g(avg_ind)); mean(block_b(avg_ind))]);
        end
    end
end

