function prog2(dir)
%Data_path = 'GrandCanyon1/' % Stitching "GrandCanyon1"
%Data_path = 'ucsb4/' % Stitching "GrandCanyon1"
%Data_path = 'glacier4/' % Stitching "GrandCanyon1"
%Data_path = 'intersection/' % Stitching "GrandCanyon1"
%Data_path = 'family_house/' % Stitching "GrandCanyon1"
img = HW3_Main_Zhenyu(dir);
imwrite([dir '_out.jpg'],img);
end