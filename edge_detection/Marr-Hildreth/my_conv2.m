function output = my_conv2(a,kernel)
%http://www.songho.ca/dsp/convolution/convolution2d_example.html

%flips
kernel1 = kernel;
for i= 1:size(kernel,1)
   kernel1(i,:) = kernel(size(kernel,1)-i+1,:); 
end

kernel2 = kernel1;
for i= 1:size(kernel,2)
   kernel2(:,i) = kernel1(:,size(kernel,2)-i+1); 
end
kernel = kernel2;

mid_r = ceil(size(kernel,1)/2);
mid_c = ceil(size(kernel,2)/2);

output = zeros(size(a,1),size(a,1));


for i = 1:size(a,1)
    for j =1:size(a,2)
        sum = 0;
        for ii=1:size(kernel,1)
            for jj = 1:size(kernel,2)
                element_a = access_elem(a,i-(mid_r-ii),j-(mid_c-jj));
                element_k = access_elem(kernel,ii,jj);
                sum = sum+element_a*element_k;
            end
            output(i,j) = sum;
        end
    end
end

    

end