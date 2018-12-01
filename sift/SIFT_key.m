function descriptorSet = SIFT_key(img)
addpath('~/matlabplugins')
%based on https://blog.csdn.net/weixin_38404120/article/details/73740612
%https://www.cnblogs.com/wangguchangqing/p/4853263.html


sigma  = 0.6;
k = 1/4;


%imshow(img)

sclaes = [];
sigmas = [];
%create scales
for i = 1:4
    sigmas(i) = sigma*(2^(i-1));
    sclaes(i) = (k^i)*sigma*(2^(i-1));
end



%create gussian kernel 高斯核
for i = 1:4
    gk{i} = createGussianKernel(5,sclaes(i));
end


%convolution on image with gussian kernel
%https://www.mathworks.com/help/matlab/ref/conv2.html
% img =gk conv2(img,gk);
% img = uint8(img);


%create Laplassian of gussian
%resize of 5 level
LOG1 = {};
for i = 1:5
    LOG1{i} = imresize(img,0.5^(i-1));
end


%for each Octave create image for different gaussian
LOG = {};
for i = 1:5
    for j = 1:4
        LOG{i}.gussian{j} = conv2(LOG1{i},gk{j});
    end
end


%create DOG
DOG = {};
for i = 1:5
    for j = 1:3
        DOG{i}.gussian{j} = LOG{i}.gussian{j+1} - LOG{i}.gussian{j};
    end
end

%search for exrtem value
extrem_min = 0.4;
extrem_max = 0.6;

extrems = {};
extrmsIdx = 1;
descriptorSet = {};
descriptorSetIdx = 1;
for i = 1:5
    process = (i/5.0)
    for gLevel = 1+1:3-1
        
        k = int32(3*1.5*sigmas(gLevel));
        
        tempImg = DOG{i}.gussian{gLevel};
        tempImg = tempImg-min(min(tempImg));
        tempImg = tempImg/max(max(tempImg));
        width = size(tempImg,1);
        height = size(tempImg,2);
        
        tempImg_upper = DOG{i}.gussian{gLevel-1};
        tempImg_lower = DOG{i}.gussian{gLevel-1};
        for m = 1+1+k:width-1-k
            for n = 1+1+k:height-1-k
                
                current = tempImg(m,n);
                
                %search for 26 pixels around
                around = tempImg_upper(m-1:m+1,n-1:n+1);
                around = [around, tempImg_lower(m-1:m+1,n-1:n+1)];
                
                around = [around(1,:), around(2,:) around(3,:)];
                
                for x = -1:1
                    for y = -1:1
                        if(~(x==0&&y==0))
                            around = [around,tempImg(m+x,n+y)];
                        end
                    end
                end
                if((current<min(around) || current > max(around))&&current>=extrem_min&&current<=extrem_max)
                    extrems{extrmsIdx}.location = [n,m];
                    extrems{extrmsIdx}.scale = sclaes(gLevel);
                    extrems{extrmsIdx}.octaive = i;
                    extrems{extrmsIdx}.extremVal = current;
                    %extrems{extrmsIdx}.gradiant.m = sqrt((tempImg(m+1,n)-tempImg(m-1,n))^2 + (tempImg(m,n+1)-tempImg(m,n-1))^2);
                    %extrems{extrmsIdx}.gradiant.theta = atan((tempImg(m,n+1)-tempImg(m,n-1)) / (tempImg(m+1,n)-tempImg(m-1,n)));
                    
                    gradiant = [];
                    gradiantIdx= 1;
                    %explore neighbors for orientation
                    
                    for x = -k:k
                        for y = -k:k
                            %if((m+x-1>=1&&m+x+1<=width&&n+y-1>=1&&n+y+1<=height))
                            gradiant(gradiantIdx,1) = sqrt((tempImg(m+x+1,n+y)-tempImg(m+x-1,n+y))^2 + (tempImg(m+x,n+y+1)-tempImg(m+x,n+y-1))^2);
                            gradiant(gradiantIdx,2) = myatan((tempImg(m+x,n+y+1)-tempImg(m+x,n+y-1)) , (tempImg(m+x+1,n+y)-tempImg(m+x-1,n+y)));
                            gradiantIdx = gradiantIdx+1;
                            %end
                        end
                    end
                    
                    %check distribution of oritation
                    BinEdges = [0:(2*3.14159)/36.0:2*3.14159];
                    h = histcounts(gradiant(:,2),BinEdges);
                    %hh = histogram(gradiant(:,2),[0:(2*3.14159)/36.0:2*3.14159]);
                    distribution = h;
                    [maxV,maxIdx] = max(distribution);
                    extrems{extrmsIdx}.gradiant.m = maxV;
                    extrems{extrmsIdx}.gradiant.theta = (BinEdges(maxIdx+1) + BinEdges(maxIdx))/2;
                    a = 1;
                    
                    
                    
                    %explore neighbors for descriptor creation
                    smallPieces = tempImg(m-k:m+k,n-k:n+k);
                    smallPieces_rot = zeros(size(smallPieces,1),size(smallPieces,2));
                    rotmat = rotmat2d(-extrems{extrmsIdx}.gradiant.theta);
                    
                    %for loop project rotation
                    for ii = 1:size(smallPieces,1)
                        for jj = 1:size(smallPieces,2)
                            projLoc = rotmat*[ii-size(smallPieces,1)/2;jj-size(smallPieces,2)/2];
                            projLoc = projLoc+[size(smallPieces,1)/2;size(smallPieces,2)/2];
                            if(projLoc(1)>=1&&projLoc(1)<=size(smallPieces,1)&&projLoc(2)>=1&&projLoc(2)<=size(smallPieces,2))
                                smallPieces_rot(ii,jj) = smallPieces(int32(projLoc(1)),int32(projLoc(2)));
                            end
                        end
                    end
                    %get descriptor from rotated image piece
                    descriptor = dscp(smallPieces_rot);
                    descriptorSet{descriptorSetIdx}.featureVector = descriptor;
                    descriptorSet{descriptorSetIdx}.location = [n,m];
                    descriptorSetIdx = descriptorSetIdx+1;
                    %current
                    extrmsIdx = extrmsIdx +1;
                end
                
            end
        end
    end
end


%TODO: Eliminating edge responses




%show extrem points
figure
for i = 1:5
    subplot(1,5,i)
    imshow(LOG{i}.gussian{1},[])
    size(LOG{i}.gussian{1})
    hold on;
    for j = 1:length(extrems)
        count = 0;
        if(extrems{j}.octaive==i)
            location = extrems{j}.location;
            plot(location(1),location(2),'o','MarkerSize',2);
            %plot orientation
            orientation_xy_d = [extrems{j}.gradiant.m*cos(extrems{j}.gradiant.theta),extrems{j}.gradiant.m*sin(extrems{j}.gradiant.theta)];
            orientation_xy = double(location)+orientation_xy_d;
            plot([location(1),orientation_xy(1)],[location(2),orientation_xy(2)])
            count = count +1;
        end
        count;
    end
end
hold off

discriptorImage = zeros(descriptorSetIdx-1,128);
for i = 1:length(descriptorSet)
    discriptorImage(i,:) = descriptorSet{i}.featureVector;
end
figure()
imshow(discriptorImage);


end




