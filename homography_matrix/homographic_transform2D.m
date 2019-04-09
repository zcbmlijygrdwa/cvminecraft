function newImage = homographic_transform2D(img,coords,target_coords)


coords_homo = [coords,ones(size(coords,1),1)]';
target_coords_homo = [target_coords,ones(size(target_coords,1),1)]';

%img(y,x,:)

% homography_matrix = target_coords_homo*pinv(coords_homo);

%DLT
homography_matrix = DLT_solve_H(coords_homo,target_coords_homo) % results from DLT is so much better than pvin

homography_matrix_inv = inv(homography_matrix);

newImage = uint8(zeros(max(target_coords(:,2)),max(target_coords(:,1)),3));

for i = 1:size(newImage,1)
    i
    for j = 1:size(newImage,2)
        newLoc = [j,i,1]';
        oldLoc = homography_matrix_inv*newLoc;
        oldLoc = oldLoc/oldLoc(3);
        if(oldLoc(1)>=1&& oldLoc(1)<size(img,2) && oldLoc(2)>=1&& oldLoc(2)<size(img,1))
        newImage(i,j,:) = img(int32(oldLoc(2)),int32(oldLoc(1)),:);
        end
    end
end