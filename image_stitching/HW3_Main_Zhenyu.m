% Homework Assignment 3: Panorama Stitching
% Zhenyu Yang
function img1_new = HW3_Main_Zhenyu(Data_path)
% %Data_path = 'GrandCanyon1/' % Stitching "GrandCanyon1"
% Data_path = 'GrandCanyon2/' % Stitching "GrandCanyon1"
% %Data_path = 'glacier4/' % Stitching "GrandCanyon1"
% Data_path = 'intersection/' % Stitching "GrandCanyon1"
% Data_path = 'family_house/' % Stitching "GrandCanyon1"
Image_separator = '_';

%==========================================================================
%% ========================================================================
% Import images from designated path (folder)
% Images are automatically classified based on their name
%--------------------------------------------------------
data_path=dir(Data_path);
file_num = length(data_path)-2;
img_info=cell(file_num,2); % {name, image index}
images=cell(file_num,1);

img_class_num = 1;
a_name = data_path(img_class_num+2).name;
a_name = a_name(1:find(a_name==Image_separator)-1);
if isempty(a_name)
    a_name = 'UnknownImage';
end
img_info{img_class_num,1} = a_name;
img_info{img_class_num,2} = 1;

for i = 2:file_num
    a_name = data_path(i+2).name; % Classify images by their names
    a_name = a_name(1:find(a_name==Image_separator)-1);
    if isempty(a_name)
        a_name = 'UnknownImage'; % When image name is unreadable
    end
    class_ind = 0;
    for j = 1:img_class_num
        if strcmpi(a_name,img_info{j,1})
            class_ind = j;
        end
    end
    if class_ind > 0 % Existing class of image
        img_info{j,2} = [img_info{j,2}, i];
    elseif class_ind == 0 % New class of image is found
        img_class_num = img_class_num + 1;
        img_info{img_class_num,1} = a_name;
        img_info{img_class_num,2} = i;
    end
end
for j = 1:img_class_num
    fprintf('Image (Type %d): [%s] contains %d images\n',j,...
        img_info{j,1}, length(img_info{j,2}));
end

%==========================================================================
for type_j = 1:img_class_num % Stitch images (same class) in target path
    %--------------------------------------------------
    disp('--------------------------------------------');
    tic
    num_of_img = length(img_info{type_j,2});
    images = cell(num_of_img, 1);
    for import_k = 1:num_of_img
        file_name=[Data_path, data_path(img_info{type_j,2}(import_k)+2).name];
        images{import_k} = imread(file_name);
    end
    
    
    %get feature for each image
    features = [];
    dataSet = cell(num_of_img,1);
    for i = 1:num_of_img
        dataSet{i}.img_c = images{i};
        dataSet{i}.img = autoGray(images{i});
        
        tempFeatures.points  = detectSURFFeatures(dataSet{i}.img);
        [des,vpts] = extractFeatures(dataSet{i}.img,tempFeatures.points);
        tempFeatures.des = des;
        
        dataSet{i}.features = tempFeatures;
    end
    
    
    img1_new = zeros(4000,8000);
    homo_global = eye(3);
    
    %draw the first image
    for i = 1:size(images{1},1)
        i
        for j = 1:size(images{1},2)
            new_i = size(img1_new,1)/2+i;
            new_j = size(img1_new,2)/2+j;
            
            img1_new(new_i,new_j) = images{1}(i,j);
        end
    end
    
    dataSet_raw = dataSet;
    
    dataSet_done = {dataSet_raw{1}};
    dataSet_done{1}.homo_global = eye(3);
    dataSet_raw = {dataSet_raw{1:0},dataSet_raw{2:end}};
    
    while(size(dataSet_raw,2)>0)
        pp = 1;
        while(pp<=size(dataSet_raw,2))
        img2 = dataSet_raw{pp}.img;
        
        indexPairsMax = [];
        matchedPoints1Max = [];
        matchedPoints2Max = [];
        idxMax = 0;
        
        for i = 1:size(dataSet_done,2)
            
            [indexPairs, matchedPoints1,matchedPoints2 ] = tryMatch(dataSet_done{i}.features, dataSet_raw{pp}.features);
            
            if(size(indexPairsMax,1)==0 || size(indexPairs,1)>size(indexPairsMax,1))
                indexPairsMax = indexPairs;
                matchedPoints1Max = matchedPoints1;
                matchedPoints2Max = matchedPoints2;
                idxMax = i;
%                         figure(2)
%         subplot(1,2,1)
%         imshow(dataSet_done{i}.img);
%         subplot(1,2,2)
%         imshow(img2);
            end
        end

        
        %figure; showMatchedFeatures(dataSet_done{idxMax}.img,img2,matchedPoints1Max,matchedPoints2Max);
        %legend('matched points 1','matched points 2');
        
        if(size(indexPairs,1)<15)
            pp = pp+1;
            continue;
        end
        data_curr = dataSet_raw{pp};
        if(pp==1)
            dataSet_raw = {dataSet_raw{:,2:end}};
        elseif(pp==size(dataSet_raw,2))
            dataSet_raw = {dataSet_raw{:,1:end-1}};
        else
            dataSet_raw = {dataSet_raw{:,1:pp-1},dataSet_raw{:,pp+1:end}};
        end
        %plotFeature
        homo = myRansac(indexPairsMax,matchedPoints1Max,matchedPoints2Max,100);
        homo_global = homo*dataSet_done{idxMax}.homo_global;
        img1_new = projection(homo_global,img1_new,img2);
        
        data_curr.homo_global = homo_global;
        imshow(img1_new,[])
        dataSet_done{size(dataSet_done,2)+1} = data_curr;
        a = 1;
        end
        pp = pp+1;
    end
end