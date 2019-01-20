function f = my_gaussian_kernel(n,s)
%s = n/240;  %240
% n: kernel size nXn
% s: sigma

%https://blog.csdn.net/jorg_zhao/article/details/52687448

x = -1/2:1/(n-1):1/2;
[Y,X] = meshgrid(x,x);
f = exp( -(X.^2+Y.^2)/(2*s^2) );
f = f / sum(f(:));
end