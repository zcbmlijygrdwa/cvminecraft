function Z = radar_f(X)

% 状态空间到测量空间的非线性映射f(x)
px = X(1);
py = X(2);
vx = X(3);
vy = X(4);

Z = [sqrt(px*px + py*py), atan(py/px), (px*vx+py*vy)/sqrt(px*px + py*py)]';

end