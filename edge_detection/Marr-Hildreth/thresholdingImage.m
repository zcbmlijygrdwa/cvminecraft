function img = thresholdingImage(img)
img = img - min(min(img));
img = 255*(img/max(max(img)));
img = img<110;
ratio = sum(sum(img))/(size(img,1)*size(img,2));
if(ratio>0.5)
    img = ~img;
end
end