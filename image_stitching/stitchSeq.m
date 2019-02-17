function [ level_info, H ] = stitchSeq(level, sch_ind, level_info,...
    pair_info, num_of_img, H)
% Recursive function to contruct the tree determing the sequence
%--------------------------------------------------------------------------
node_ind = find(level_info(:,3) == sch_ind);
level = level + 1;
for i = 1:length(node_ind)
    [r, ~] = find( pair_info == node_ind(i) );
    if ~isempty(r)
        p = pair_info(r,:);
        child_ind = unique(p(p ~= node_ind(i)));
        for j = 1:length(child_ind)
            H{child_ind(j),level-1} = [child_ind(j),node_ind(i)];
        end
        level_info(child_ind,1) = level;
        level_info(child_ind,2) = node_ind(i);
        sch_ind = sch_ind + 1;
        level_info(child_ind,3) = sch_ind;
        if sum(level_info(:,1) > 0) >= num_of_img
            return
        end        
        unsearch_ind = setdiff(1:size(pair_info,1), r);
        [ level_info, H ] = stitchSeq( level, sch_ind,level_info,...
            pair_info(unsearch_ind,:), num_of_img, H );
    end
end
end

