function visulization(img,img1,img2,img3,img4,img5)

figure()

subplot(2,3,1)
imshow(img)
title('original image')

subplot(2,3,2)
imshow(img1)
title('sigma = 3')

subplot(2,3,3)
imshow(img2)
title('sigma = 6')

subplot(2,3,4)
imshow(img3)
title('sigma = 12')

subplot(2,3,5)
imshow(img4)
title('sigma = 24')

subplot(2,3,6)
imshow(img5)
title('sigma = 48')
end