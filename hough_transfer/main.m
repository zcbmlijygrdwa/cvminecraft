%https://blog.csdn.net/ws_20100/article/details/51159434

close all
clear all



%r = x*cos(theta)+y*sin(theta);


res_theta = 0.005;
res_r = 0.005;

pie = 3.141592653;
size_theta = ceil(2*pie/res_theta);
size_r = ceil(10/res_r);

img = zeros(size_r,size_theta);
xs = zeros(1,1000);
ys = zeros(1,length(xs));

for i = 1:length(xs)
i
x = i;
y = 2*x+rand;
xs(i) = x;
ys(i) = y;

for theta = res_theta:res_theta:2*pie
    r = x*cos(theta)+y*sin(theta);
    x_i = theta/res_theta;
    y_i = size_r/2+r/res_r;
    x_i = int32(x_i);
    y_i = int32(y_i);
    if(x_i>=1&&x_i<=size_theta&&y_i>=1&&y_i<=size_r)
        img(y_i,x_i) = img(y_i,x_i) + 5;
    end
end
end




maxV = 0;
max_location = [];
for i = 1:size(img,1)
    for j = 1:size(img,2)
        if(img(i,j)>maxV)
           maxV = img(i,j);
           max_location = [i,j];
        end
    end
end

theta = max_location(2)*res_theta
r = (max_location(1) - size_r/2)*res_r

subplot(1,2,1)
imshow(img,[])
hold on
plot(max_location(2),max_location(1),'o');
hold off;

x2 = 0:length(xs);
y2 = (r - cos(theta)*x2)/sin(theta);

subplot(1,2,2)
plot(x2,y2)
hold on
plot(xs,ys,'o')
hold off
pbaspect([1 1 1])