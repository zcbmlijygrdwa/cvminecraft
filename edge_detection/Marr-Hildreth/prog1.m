function imgs = prog1(impath)
% http://www.cs.ucsb.edu/~cs181b/hw/prog1.pdf
% https://blog.csdn.net/bettyshasha/article/details/51757185

ifUseMyConv2 = false;  %if use the convolution function define by myself

img = imread(impath);
tokens = strsplit(impath,'/');
tokens = strsplit(tokens{end},'.');
fileName = tokens{1};
%img = imresize(img,0.5);

%convert to grayscale
if(size(img,3)>1)
    img = rgb2gray(img);
end
img = double(img);
img = img/255;


%create gaussian kernels,cover at least 1 sigma: size == sigma
gk3 = my_gaussian_kernel(3,0.013);
gk6 = my_gaussian_kernel(6,0.025);
gk12 = my_gaussian_kernel(12,0.05);
gk24 = my_gaussian_kernel(24,0.1);
gk48 = my_gaussian_kernel(48,0.12);

%create a laplacian kernel
lk = my_laplacian_kernel();

% convolve the pyramid of the Gaussian masks with a Laplacian operator mask to generate the LoG (Laplacian of a Gaussian) or the "Mexican Hat" operator at five different scales.
if(ifUseMyConv2)
    
    log3 = my_conv2(gk3,lk);
    log6 = my_conv2(gk6,lk);
    log12 = my_conv2(gk12,lk);
    log24 = my_conv2(gk24,lk);
    log48 = my_conv2(gk48,lk);
else
    
    log3 = conv2(gk3,lk, 'same');
    log6 = conv2(gk6,lk, 'same');
    log12 = conv2(gk12,lk, 'same');
    log24 = conv2(gk24,lk, 'same');
    log48 = conv2(gk48,lk, 'same');
end

% convolve the LoG operators with your input image
if(ifUseMyConv2)
    img_conv_3 = my_conv2(img,log3);
    img_conv_6 = my_conv2(img,log6);
    img_conv_12 = my_conv2(img,log12);
    img_conv_24 = my_conv2(img,log24);
    img_conv_48 = my_conv2(img,log48);
else
    img_conv_3 = conv2(img,log3, 'same');
    img_conv_6 = conv2(img,log6, 'same');
    img_conv_12 = conv2(img,log12, 'same');
    img_conv_24 = conv2(img,log24, 'same');
    img_conv_48 = conv2(img,log48, 'same');
end

% detect zero-crossings
img_out_3 = my_zero_crossing_detector(img_conv_3);
img_out_6 = my_zero_crossing_detector(img_conv_6);
img_out_12 = my_zero_crossing_detector(img_conv_12);
img_out_24 = my_zero_crossing_detector(img_conv_24);
img_out_48 = my_zero_crossing_detector(img_conv_48);

%thresholding image
img_out_3 = thresholdingImage(img_out_3);
img_out_6 = thresholdingImage(img_out_6);
img_out_12 = thresholdingImage(img_out_12);
img_out_24 = thresholdingImage(img_out_24);
img_out_48 = thresholdingImage(img_out_48);


%show results
%visulization(img,img_out_3,img_out_6,img_out_12,img_out_24,img_out_48);
% figure(),imshow(img_out_3)
% figure(),imshow(img_out_6)
% figure(),imshow(img_out_12)
% figure(),imshow(img_out_24)
% figure(),imshow(img_out_48)

% Output five binary edge maps at five different scales
imgs{1} = img_out_3;
imgs{2} = img_out_6;
imgs{3} = img_out_12;
imgs{4} = img_out_24;
imgs{5} = img_out_48;
for i = 1:length(imgs)
    str = ['output/',fileName,'_',num2str(i-1),'.bmp'];
    disp(['Writing to:[',str,']']);
    imwrite(imgs{i},str);
end

