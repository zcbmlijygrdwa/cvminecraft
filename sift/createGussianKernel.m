function gk = createGussianKernel(size,sigma)
pie = 3.141592653;

k = 1/(2*pie*sigma*sigma);

gk = zeros(size,size);
mid = (1+size)/2;
for i = 1:size
    for j = 1:size
        c = -((  (mid-i)^2 + (mid-j)^2  )/(2*sigma*sigma));
        gk(i,j) = k*exp(c);
    end
end

end