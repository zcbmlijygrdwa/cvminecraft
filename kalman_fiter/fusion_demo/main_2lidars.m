clear all;
close all;

%https://blog.csdn.net/Young_Gy/article/details/78468153

myLocation.x = 0;
myLocation.y = 0;
myRadarLocation_prev = myLocation;

lidar_noise_param.mu = 0;
lidar_noise_param.sigma = 18.4;

lidar2_noise_param.mu = 0;
lidar2_noise_param.sigma = lidar_noise_param.sigma;

%delta t
d_t = 1;

%kalman states: [px,py,vx,vy]
X_curr = [0.1,0.2,0.1,0.2]';
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
        
R_lidar2 = [lidar2_noise_param.sigma*lidar2_noise_param.sigma 0
            0 lidar2_noise_param.sigma*lidar2_noise_param.sigma];
          
          

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
       
       
H_lidar2 = [1 0 0 0
           0 1 0 0];
       
sim_length = 1000;

%loc_data = zeros(sim_length,1);

lidar_location_set = [];
lidar2_location_set = [];
kf_fuse_location_set = [];
true_location_set = [];


for i = 1:sim_length

myLocation.x = -35+35*cos(i/250);
myLocation.y = 50*sin(i/200);

% myLocation.x = i;
% myLocation.y = 2*i;


lidar_resp = lidar_response(myLocation,lidar_noise_param);
lidar2_resp = lidar_response(myLocation,lidar2_noise_param);

lidar_location_set = [lidar_location_set; lidar_resp.location.x lidar_resp.location.y];
lidar2_location_set = [lidar2_location_set; lidar2_resp.location.x lidar2_resp.location.y];

true_location_set = [true_location_set; myLocation.x myLocation.y];



Z_lidar = [lidar_resp.location.x lidar_resp.location.y]';
Z_lidar2 = [lidar2_resp.location.x lidar2_resp.location.y]';
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


%=================================
%   update lidar2
%=================================


%refine current
K_lidar2 = P_next*(H_lidar2')*inv(H_lidar2*P_next*H_lidar2' + R_lidar2);
X_curr = X_next + K_lidar2*(Z_lidar2 - H_lidar2*X_next);
P_curr = (eye(4)-K_lidar2*H_lidar2)*P_next;

%predict
X_next = F*X_curr
P_next = F*P_curr*F'+Q;



kf_fuse_location_set = [kf_fuse_location_set; X_curr(1) X_curr(2)];

myRadarLocation_prev = lidar2_resp.location;
end



%draw
figure(1)
subplot(1,4,1)
hold on;
scatter(lidar_location_set(:,1),lidar_location_set(:,2),'r.');
scatter(lidar2_location_set(:,1),lidar2_location_set(:,2),'b.');
scatter(kf_fuse_location_set(:,1),kf_fuse_location_set(:,2),'g.');
scatter(true_location_set(:,1),true_location_set(:,2),'k.');
legend('lidar','lidar2','lidar-kf','groudtruth')
hold off;
xlim([-120,60])
ylim([-100,100])

subplot(1,4,2)
scatter(lidar_location_set(:,1),lidar_location_set(:,2),'r.');
legend('lidar')
xlim([-120,60])
ylim([-100,100])

subplot(1,4,3)
scatter(lidar2_location_set(:,1),lidar2_location_set(:,2),'b.');
legend('lidar2')
xlim([-120,60])
ylim([-100,100])

subplot(1,4,4)
scatter(kf_fuse_location_set(:,1),kf_fuse_location_set(:,2),'g.');
legend('fuse-kf')
xlim([-120,60])
ylim([-100,100])