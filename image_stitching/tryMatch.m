function [ indexPairs, matchedPoints1,matchedPoints2 ] = tryMatch( imgf1, imgf2)

indexPairs = matchFeatures(imgf1.des,imgf2.des,'Unique',true,'MaxRatio',0.4) ;
matchedPoints1 = imgf1.points(indexPairs(:,1));
matchedPoints2 = imgf2.points(indexPairs(:,2));

end