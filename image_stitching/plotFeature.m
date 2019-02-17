function plotFeature( img1, img2, xy1, xy2 )
% Plot---------------------------------------------------------------------
pt_num = size(xy1, 2);

figure('Position', [100,150,1000,600])
subplot(1,2,1)
imshow(uint8(img1))
hold on
for i = 1:pt_num
    plot(xy1(1,i), xy1(2,i), 'r+', 'LineWidth',1)
    text(xy1(1,i), xy1(2,i), sprintf('%d',i),...
        'Color','cyan', 'fontsize', 12)
end
hold off
title('Image A')

subplot(1,2,2)
imshow(uint8(img2))
hold on
for i = 1:pt_num
    plot(xy2(1,i), xy2(2,i), 'r+', 'LineWidth',1)
    text(xy2(1,i), xy2(2,i), sprintf('%d',i),...
        'Color','cyan', 'fontsize', 12)
end
hold off
title('Image B')
drawnow;
end