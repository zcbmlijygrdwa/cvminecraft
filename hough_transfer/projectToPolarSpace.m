%https://blog.csdn.net/ws_20100/article/details/51159434

close all
clear all



%r = x*cos(theta)+y*sin(theta);


res_theta = 0.01;
res_r = 0.01;

pie = 3.141592653;
size_theta = ceil(2*pie/res_theta);
size_r = ceil(10/res_r);

img = zeros(size_r,size_theta);


x = 1;
y = 2;
for theta = res_theta:res_theta:2*pie
   r = x*cos(theta)+y*sin(theta);
   x_i = theta/res_theta;
   y_i = size_r/2+r/res_r;
   x_i = int32(x_i)
   y_i = int32(y_i)
   img(y_i,x_i) = img(y_i,x_i) + 1;
end


x = 3;
y = 2;
for theta = res_theta:res_theta:2*pie
   r = x*cos(theta)+y*sin(theta);
   x_i = theta/res_theta;
   y_i = size_r/2+r/res_r;
   x_i = int32(x_i)
   y_i = int32(y_i)
   img(y_i,x_i) = img(y_i,x_i) + 1;
end


x = -1;
y = 2;
for theta = res_theta:res_theta:2*pie
   r = x*cos(theta)+y*sin(theta);
   x_i = theta/res_theta;
   y_i = size_r/2+r/res_r;
   x_i = int32(x_i)
   y_i = int32(y_i)
   img(y_i,x_i) = img(y_i,x_i) + 1;
end

imshow(img)