function f = my_log_kernel(n,s)

% n: kernel size nXn
% s: sigma

%https://blog.csdn.net/jorg_zhao/article/details/52687448

x = -1/2:1/(n-1):1/2;
[Y,X] = meshgrid(x,x);
f = ((X.^2+Y.^2-2*s^2)/(s^4))*exp( -(X.^2+Y.^2)/(2*s^2) );
f = f / abs(sum(f(:)));
end