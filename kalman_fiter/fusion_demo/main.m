clear all;
close all;

%https://blog.csdn.net/Young_Gy/article/details/78468153

myLocation.x = 0;
myLocation.y = 0;
myRadarLocation_prev = myLocation;

lidar_noise_param.mu = 0;
lidar_noise_param.sigma = 18.8;

radar_noise_param.mu = 0;
radar_noise_param.sigma = 0.05;

%delta t
d_t = 1;

%kalman states: [px,py,vx,vy]
X_curr = [0,0,0,0]';
X_next = X_curr;

%state error covariance
P_curr = [0 0 0 0
     0 0 0 0
     0 0 0 0
     0 0 0 0];
P_next = P_curr;

%covariance of the process noise
%https://blog.csdn.net/Young_Gy/article/details/78468153
Q = [1/4 0 1/2 0
     0 1/4 0 1/2
     1/2 0 1 0
     0 1/2 0 1];
 
 Q = eye(4,4)*0.01;

%noise covariance of the sensors
R_lidar = [lidar_noise_param.sigma*lidar_noise_param.sigma 0
            0 lidar_noise_param.sigma*lidar_noise_param.sigma];
        
R_radar = [0.09 0 0
              0 0.0009 0
              0 0 0.09];

          
          

%state-transition matrix: 
%x_next = F*x_curr
%P_next = F*P_curr*F' + Q
%assuming velocity not change
F = [1 0 d_t 0
     0 1 0 d_t
     0 0 1 0
     0 0 0 1];
 

% the observation matrix
% delta_Z = Z - H*X_next
% P_curr = (I - K*H)*P_next 
H_lidar = [1 0 0 0
           0 1 0 0];
       
       
%https://medium.com/@mithi/sensor-fusion-and-object-tracking-using-an-extended-kalman-filter-algorithm-part-2-cd20801fbeff
px = X_next(1);
py = X_next(2);
vx = X_next(3);
vy = X_next(4);
H_radar_jacobian = [px/(sqrt(px*px+py*py)) py/(sqrt(px*px+py*py)) 0 0
                    -py/(px*px+py*py) -px/(px*px+py*py) 0 0
                    py*(vx*py-vy*px)/((px*px+py*py)*sqrt(px*px+py*py)) px*(vy*px-vx*py)/((px*px+py*py)*sqrt(px*px+py*py)) px/(sqrt(px*px+py*py)) py/(sqrt(px*px+py*py))];
       


sim_length = 500;

%loc_data = zeros(sim_length,1);

lidar_location_set = [];
radar_location_set = [];
kf_lidar_location_set = [];
true_location_set = [];


for i = 1:sim_length

% myLocation.x = 6+5*cos(i/20);
% myLocation.y = 6+5*sin(i/18);

myLocation.x = i;
myLocation.y = 2*i;


lidar_resp = lidar_response(myLocation,lidar_noise_param);
radar_resp = radar_response(myLocation, myRadarLocation_prev, radar_noise_param);

lidar_location_set = [lidar_location_set; lidar_resp.location.x lidar_resp.location.y];
radar_location_set = [radar_location_set; radar_resp.location.x radar_resp.location.y];

true_location_set = [true_location_set; myLocation.x myLocation.y];



Z_lidar = [lidar_resp.location.x lidar_resp.location.y]';
Z_radar = [radar_resp.rho radar_resp.phi radar_resp.d_pho]';
%process data


%=================================
%   update lidar
%=================================


%refine current
K_lidar = P_next*(H_lidar')*inv(H_lidar*P_next*H_lidar' + R_lidar);
X_curr = X_next + K_lidar*(Z_lidar - H_lidar*X_next);
P_curr = (eye(4)-K_lidar*H_lidar)*P_next;

%predict
X_next = F*X_curr

P_next = F*P_curr*F'+Q;

temp_loc_data.kf_lidar_location.x = X_curr(1);
temp_loc_data.kf_lidar_location.y = X_curr(2);
kf_lidar_location_set = [kf_lidar_location_set; X_curr(1) X_curr(2)];

%=================================
%   update radar
%=================================


%refine current

% px = X_next(1);
% py = X_next(2);
% vx = X_next(3);
% vy = X_next(4);
% H_radar_jacobian = [px/(sqrt(px*px+py*py)) py/(sqrt(px*px+py*py)) 0 0
%                     -py/(px*px+py*py) -px/(px*px+py*py) 0 0
%                     py*(vx*py-vy*px)/((px*px+py*py)*sqrt(px*px+py*py)) px*(vy*px-vx*py)/((px*px+py*py)*sqrt(px*px+py*py)) px/(sqrt(px*px+py*py)) py/(sqrt(px*px+py*py))];
%        
% 
% 
% K_radar = P_next*(H_radar_jacobian')*inv(H_radar_jacobian*P_next*H_radar_jacobian' + R_radar);
% X_curr = X_next + K_radar*(Z_radar - H_radar_jacobian*X_next);
% P_curr = (eye(4)-K_radar*H_radar_jacobian)*P_next;
% 
% %predict
% X_next = F*X_curr
% P_next = F*P_curr*F'+Q;





myRadarLocation_prev = radar_resp.location;
end



%draw
figure(1)
hold on;
scatter(lidar_location_set(:,1),lidar_location_set(:,2),'r.');
%scatter(radar_location_set(:,1),radar_location_set(:,2),'b.');
scatter(kf_lidar_location_set(:,1),kf_lidar_location_set(:,2),'g.');
scatter(true_location_set(:,1),true_location_set(:,2),'k.');
legend('lidar','lidar-kf','groudtruth')
hold off;