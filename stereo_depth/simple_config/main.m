close all;
clear all;

% mathcing implementation Sum of Absolute Difference(SAD).

ifVis = false;

img1 = imread('art/pentagon_left.bmp');
if(size(img1,3)>1)
    img1 = rgb2gray(img1);
end
%img1 = imresize(img1,0.8);
img1 = double(img1);

img2 = imread('art/pentagon_right.bmp');
if(size(img2,3)>1)
    img2 = rgb2gray(img2);
end
%img2 = imresize(img2,0.8);
img2 = double(img2);

for cubeSize_t = 2:5:50
cubeSize = 4;
cubeSize = cubeSize_t;
cubeSize_d = cubeSize-1;

depth_map = zeros(size(img1,1)-cubeSize_d,size(img1,2)-cubeSize_d);

for i = 1:size(img1,1)-cubeSize_d
    cubeSize,i
    for j = 1:size(img1,2)-cubeSize_d
        
        temp_piece = img1(i:i+cubeSize_d,j:j+cubeSize_d);
        
        k_min = size(img1,2);
        diff_min = sum(sum(temp_piece));
        try_piece_min = [];
        %at the same scanline, search for similar block in img2
        for k =  j+1:size(img2,2)-cubeSize_d
            try_piece = img2(i:i+cubeSize_d,k:k+cubeSize_d);
            
            
            %            subplot(1,2,1);
            %            imshow(uint8(temp_piece));
            %            title('temp_piece');
            %
            %            subplot(1,2,2);
            %            imshow(uint8(try_piece));
            %            title('try_piece');
            
            diff = sum(sum(abs(temp_piece-try_piece)));
            if(diff<diff_min)
                diff_min = diff;
                k_min = k;
                try_piece_min = try_piece;
            end
            
            a = 1;
        end
        
        depth_map(i,j) = k_min-j;
        
        
        
        if(ifVis)
            subplot(2,4,1:2)
            imshow(depth_map,[])
            
            subplot(2,4,3)
            imshow(temp_piece,[])
            
            subplot(2,4,4)
            imshow(try_piece_min,[])
            
            img1_rgb = uint8(cat(3, img1, img1, img1));
            img1_rgb = addRect(img1_rgb,i,j,cubeSize);
            subplot(2,4,5:6)
            imshow(img1_rgb)
            
            img2_rgb = uint8(cat(3, img2, img2, img2));
            img2_rgb = addRect(img2_rgb,i,k_min,cubeSize);
            subplot(2,4,7:8)
            imshow(img2_rgb)
            
            k_min-j
            pause(0.001);
        end
        
    end
    
    
    
    
    
end
imwrite(uint8(depth_map),['test_wsize_',num2str(cubeSize),'.jpg']);
end
imshow(depth_map,[])


