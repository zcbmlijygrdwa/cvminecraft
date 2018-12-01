function discriptor = dscp(img)
xm = int8(size(img,1)/2);
ym = int8(size(img,2)/2);
discriptor = zeros(1,128);
imgs = {};
imgsIdx = 0;
for i = -1:2
    for j = -1:2
        distribution = zeros(1,8);
        if((img(xm+i,ym+j+1)-img(xm+i,ym+j-1))~=0 || (img(xm+i+1,ym+j)-img(xm+i-1,ym+j))~=0)
            ang = myatan((img(xm+i,ym+j+1)-img(xm+i,ym+j-1)) , (img(xm+i+1,ym+j)-img(xm+i-1,ym+j)));
            data = floor(ang/(2*3.141592653)*7);
            distribution(data+1) = 1;
        end
        discriptor(1,1+imgsIdx*8:8+imgsIdx*8) = distribution;
        imgsIdx = imgsIdx+1;
    end
end
end