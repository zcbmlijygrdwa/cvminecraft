function [ H ] = estimateHomography( xy1_homo, xy2_homo )
pt_num = size(xy1_homo, 2);
% Estimate the transformation function by DLT
%--------------------------------------------------------------------------
T1 = featureHartleyNormalize( xy1_homo(1:2,:) );
T2 = featureHartleyNormalize( xy2_homo(1:2,:) );

xy1_norm = T1 * xy1_homo;
xy2_norm = T2 * xy2_homo;

A = zeros(pt_num * 2, 9);
for i = 1:pt_num
    A((i-1)*2+1,:) = [xy1_norm(1,i), xy1_norm(2,i), 1, 0, 0, 0,...
        -xy1_norm(1,i)*xy2_norm(1,i), -xy1_norm(2,i)*xy2_norm(1,i),...
        -xy2_norm(1,i)];
    A((i-1)*2+2,:) = [0, 0, 0, xy1_norm(1,i), xy1_norm(2,i), 1,...
        -xy1_norm(1,i)*xy2_norm(2,i), -xy1_norm(2,i)*xy2_norm(2,i),...
        -xy2_norm(2,i)];
end

[~, ~, V] = svd(A);

h = V(:,end)/norm(V);
H_norm = reshape(h, [3,3])';

H = T2 \ H_norm * T1;
H = H ./ H(end,end);
end

%% Local function
function T = featureHartleyNormalize( features )
    % compute the Hartley Normalize matrix based on the given features
    % features are 2xN matrix with the format [x; y] for each column
    
    % Get the size
    len = length(features(1,:));
    
    % Expand the input matrix into homogeneous coordinates by adding a row
    % of 1
    m = [features; ones(1,len)];
    
    % Compute the centroid
    centroid = mean(m,2);
    
    % Compute average distance
    dists = sqrt(sum((m - repmat(centroid,1,size(m,2))).^2,1));
    mean_dist = mean(dists);
    
    % Compute T
    T = [sqrt(2)/mean_dist 0 -sqrt(2)/mean_dist*centroid(1);...
         0 sqrt(2)/mean_dist -sqrt(2)/mean_dist*centroid(2);...
         0 0 1];
end