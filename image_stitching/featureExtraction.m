function [ points1,points2 ] = featureExtraction( img1, img2)


points1  = detectSURFFeatures(img1);
points2  = detectSURFFeatures(img2);

end