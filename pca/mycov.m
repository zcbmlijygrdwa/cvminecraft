%https://blog.csdn.net/qq_34562093/article/details/80567754

function covs = mycov(data)

%data: n*m  : m dimension data
% |  creature1  |  creature2  |  creature3  |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |
% |     data    |     data    |    data     |

%above generates m*m covariance matrix


%another approach: remove mean, and cov = (1/(n-1))*(data' * data); %https://blog.csdn.net/hustqb/article/details/78394058

numOfCategory = size(data,2);
numOfData = size(data,1);
%calculate mean for each category
means = zeros(1,numOfCategory);

for i = 1:numOfCategory
    means(i) = mean(data(:,i));
end

%calculate co-variance for each category
covs = zeros(numOfCategory,numOfCategory);
for i = 1:numOfCategory
    for j = 1:numOfCategory
        sum = 0;
        for k = 1:numOfData
            sum = sum + (data(k,i)-means(i))*(data(k,j)-means(j));
        end
        covs(i,j) = sum/(numOfData-1);
    end
end
covs = covs;
end