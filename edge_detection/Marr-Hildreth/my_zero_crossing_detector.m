function output = my_zero_crossing_detector(img)
% https://en.wikipedia.org/wiki/Zero_crossing
% https://blog.csdn.net/bettyshasha/article/details/51757185
% https://blog.csdn.net/u013263891/article/details/83864948
rows = size(img,1);
cols = size(img,2);

output = zeros(rows,cols);

%sliding 3X3 window
for i = 2:rows-1
    for j = 2:cols-1
        mid = img(i,j);
        
        left = img(i,j-1);
        right = img(i,j+1);
        
        up = img(i-1,j);
        down = img(i+1,j);
        
        up_left = img(i-1,j-1);
        up_right = img(i-1,j+1);
        
        down_left = img(i+1,j-1);
        down_right = img(i+1,j+1);
        
        if(mid*left<=0 ...
                ||mid*right<=0 ...
                ||mid*up<=0 ...
                ||mid*down<=0 ...
                ||mid*up_left<=0 ...
                ||mid*up_right<=0 ...
                ||mid*down_left<=0 ...
                ||mid*down_right<=0 ...
                )
                output(i,j) = mid;
        else
            output(i,j) = 0;
        end
        
%         if(zero_crossing_evaluator(left,mid,right)...
%                 ||zero_crossing_evaluator(up,mid,down))
%                 ||zero_crossing_evaluator(down_left,mid,up_right)...
%                 ||zero_crossing_evaluator(up_left,mid,down_right))
%                 output(i,j) = mid;
%         else
%             output(i,j) = 0;
%         end
        
        
    end
end
end