function homo_min = myRansac(indexPairs,matchedPoints1,matchedPoints2,iters)

%use four points to do ransac
isInit = false;
err_min = 0;
homo_min = [];
numOfPoints = 4;
for i = 1:iters
    points1 = zeros(numOfPoints,2);
    points2 = zeros(numOfPoints,2);
    
    randomIdxs = randperm(size(indexPairs,1),numOfPoints);
    for j = 1:numOfPoints
        points1(j,:) = matchedPoints1.Location(randomIdxs(j),:);
        points2(j,:) = matchedPoints2.Location(randomIdxs(j),:);
    end
    points1 = points1';
    points2 = points2';
    
    temp_homo = myHomo(points1,points2);
    points2_predic =[ homoTransform(temp_homo,matchedPoints1.Location')]';
    diff = points2_predic-matchedPoints2.Location;
    err = sum(sum(abs(diff)));
    if(~isInit || err<err_min)
        err_min = err;
        homo_min = temp_homo;
        isInit = true;
    end
    err_min
end

end