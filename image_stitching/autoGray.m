function img = autoGray(img)
if size(img, 3) > 1
    img = rgb2gray(img);
end
end