function output = zero_crossing_evaluator(a,b)

% assuming order [a,b,c]

%zero crossing if a>0,b<0 or a<0,b>0,c<0

threshold = 0;

if((a>0&&b<0&&c>0) || (a<0&&b>0&&c<0))
    
    %further check the threshold
    if(abs(a-b)>threshold && abs(c-b)>threshold)
        output = true;
    else
        output = false;
    end
else
    output = false;
end

end