close all
clear all
stepSize = 1;
p = [0,0,0
    0,1,0
    1,0,0];

p1 = p(1,:);

p2 = p(2,:);

p3 = p(3,:);

a1 = p2-p1;
a2 = p3-p1;


n = cross(a1,a2);

dimage = zeros(20,20);

img = imread('test.jpg');
img = rgb2gray(img);
img = imresize(img,0.2);
img = img/10;



for i=1:size(img,1)
    for j=1:size(img,2)
        dimage(i,j) = sqrt(i+j)*sqrt(i);
    end
end

dimage = double(img);

output = {};
outputIdx = 1;
for i=1:size(img,1)
    for j=1:size(img,2)
        if(i+stepSize<=size(img,1) && j+stepSize <=size(img,2))
            p1 = [i,j,dimage(i,j)];
            p2 = [i+stepSize,j,dimage(i+stepSize,j)];
            p3 = [i,j+stepSize,dimage(i,j+stepSize)];
            
            a1 = p2-p1;
            a2 = p3-p1;
            
            n = cross(a1,a2);
            n = n/norm(n);
            output{outputIdx}.location = p1;
            output{outputIdx}.n = n;
            outputIdx = outputIdx+1;
        end
    end
end



%show
x = zeros(1,size(img,1)*size(img,2));
y = zeros(1,size(img,1)*size(img,2));
z = zeros(1,size(img,1)*size(img,2));
idx = 1;
for i=1:size(img,1)
    for j=1:size(img,2)
        x(idx) = i;
        y(idx) = j;
        z(idx) = dimage(i,j);
        idx = idx+1;
    end
end

scatter3(x,y,z);
daspect([1 1 1])
hold on;
for i = 1:length(output)
    %mArrow3(output{i}.location,output{i}.location+output{i}.n, 'stemWidth', 0.05);
    v1=output{i}.location;
    v2=output{i}.location+output{i}.n*3;
    v=[v2;v1];
    plot3(v(:,1),v(:,2),v(:,3),'r')
end
hold off;

figure()
imshow(img,[])