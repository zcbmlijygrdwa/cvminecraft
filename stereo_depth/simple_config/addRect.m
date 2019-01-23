function img = addRect(img,ii,jj,cubeSize)

for i = ii:ii+cubeSize-1
    
    for k = 1:cubeSize
        img(i,jj,1) = 255;
        img(i,jj,2) = 0;
        img(i,jj,3) = 0;
        
        img(i,jj+cubeSize-1,1) = 255;
        img(i,jj+cubeSize-1,2) = 0;
        img(i,jj+cubeSize-1,3) = 0;
    end
end

for j = jj:jj+cubeSize-1
    
    for k = 1:cubeSize
        img(ii,j,1) = 255;
        img(ii,j,2) = 0;
        img(ii,j,3) = 0;
        
        img(ii+cubeSize-1,j,1) = 255;
        img(ii+cubeSize-1,j,2) = 0;
        img(ii+cubeSize-1,j,3) = 0;
    end
end
    

end