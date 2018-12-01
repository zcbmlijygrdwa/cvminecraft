function showFeatureCorr(img1,img2,matchedPoints1,matchedPoints2)

compareImage = zeros(size(img1,1)+size(img2,1),max(size(img1,1),size(img2,1)));
compareImage(1:size(img1,1),1:size(img1,2)) = img1;
compareImage(size(img1,1)+1:size(img1,1)+size(img2,1),1:size(img2,2)) = img2;
figure();
imshow(compareImage,[]);
hold on
for i = 1:size(matchedPoints1,1)

%plot(matchedPoints1(i,1),matchedPoints1(i,2),'o')
%plot(matchedPoints2(i,1),size(img1,1)+matchedPoints2(i,2),'o')
plot([matchedPoints1(i,1),matchedPoints2(i,1)],[matchedPoints1(i,2),size(img1,1)+matchedPoints2(i,2)])
end
hold off

end