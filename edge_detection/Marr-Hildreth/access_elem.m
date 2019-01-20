function out = access_elem(a,i,j)
    if(i>=1&&i<=size(a,1) && j>=1&&j<=size(a,2))
        out =  a(i,j);
    else
        out =  0;
    end
    
end