clear all;
close all;

myLocation.x = 0;
myLocation.y = 0;
myRadarLocation_prev = myLocation;

lidar_noise_param.mu = 0;
lidar_noise_param.sigma = 0.8;

radar_noise_param.mu = 0;
radar_noise_param.sigma = 0.1;




sim_length = 100;

%loc_data = zeros(sim_length,1);


for i = 1:sim_length

myLocation.x = 6+5*cos(i/20);
myLocation.y = 6+5*sin(i/18);

lidar_resp = lidar_response(myLocation,lidar_noise_param);
radar_resp = radar_response(myLocation, myRadarLocation_prev, radar_noise_param);


temp_loc_data.lidar_loc = lidar_resp.location;
temp_loc_data.radar_loc = radar_resp.location;
temp_loc_data.true_loc = myLocation;
loc_data(i) = temp_loc_data;

myRadarLocation_prev = radar_resp.location;
end



%draw
figure(1)
hold on;
for i = 1:sim_length
    scatter(loc_data(i).lidar_loc.x,loc_data(i).lidar_loc.y,'r.');
    scatter(loc_data(i).radar_loc.x,loc_data(i).radar_loc.y,'b.');
    scatter(loc_data(i).true_loc.x, loc_data(i).true_loc.y,'k.');
end
hold off;