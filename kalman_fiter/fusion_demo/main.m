clear all;
close all;

%https://blog.csdn.net/Young_Gy/article/details/78468153

myLocation.x = 0;
myLocation.y = 0;
myRadarLocation_prev = myLocation;

lidar_noise_param.mu = 0;
lidar_noise_param.sigma = 4.4;

radar_noise_param.mu = 0;
radar_noise_param.sigma = 0.05;

%delta t
d_t = 1;

%kalman states: [px,py,vx,vy]
X_curr = [ 99.1391 , 111.5188,0.1,0.2]';
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
 
 Q = eye(4,4)*0.000001;

%noise covariance of the sensors
R_lidar = [lidar_noise_param.sigma*lidar_noise_param.sigma 0
            0 lidar_noise_param.sigma*lidar_noise_param.sigma];
        
R_radar = [radar_noise_param.sigma*radar_noise_param.sigma 0 0
              0 radar_noise_param.sigma*radar_noise_param.sigma 0
              0 0 radar_noise_param.sigma*radar_noise_param.sigma];

          
          

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
       


sim_length = 1000;

%loc_data = zeros(sim_length,1);

lidar_location_set = [];
radar_location_set = [];
kf_lidar_location_set = [];
true_location_set = [];


for i = 1:sim_length

myLocation.x = 65+35*cos(i/250);
myLocation.y = 114+50*sin(i/200);

% myLocation.x = i;
% myLocation.y = 2*i;


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


%=================================
%   update radar
%=================================


%refine current

px = X_next(1);
py = X_next(2);
vx = X_next(3);
vy = X_next(4);
H_radar_jacobian = [px/(sqrt(px*px+py*py)) py/(sqrt(px*px+py*py)) 0 0
                    -py/(px*px+py*py) px/(px*px+py*py) 0 0
                    py*(vx*py-vy*px)/((px*px+py*py)*sqrt(px*px+py*py)) px*(vy*px-vx*py)/((px*px+py*py)*sqrt(px*px+py*py)) px/(sqrt(px*px+py*py)) py/(sqrt(px*px+py*py))];
       


K_radar = P_next*(H_radar_jacobian')*inv(H_radar_jacobian*P_next*H_radar_jacobian' + R_radar);




%X_curr = X_next + K_radar*(Z_radar - H_radar_jacobian*X_next);
% Here, because of EKF, delta_Z = Z_radar-f(X_next)
% rather than: Z_radar - H_radar_jacobian*X_next
X_curr = X_next + K_radar*(Z_radar - radar_f(X_next));

P_curr = (eye(4)-K_radar*H_radar_jacobian)*P_next;

%predict
X_next = F*X_curr
P_next = F*P_curr*F'+Q;



kf_lidar_location_set = [kf_lidar_location_set; X_curr(1) X_curr(2)];

myRadarLocation_prev = radar_resp.location;
end



%draw
figure(1)
subplot(1,4,1)
hold on;
scatter(lidar_location_set(:,1),lidar_location_set(:,2),'r.');
scatter(radar_location_set(:,1),radar_location_set(:,2),'b.');
scatter(kf_lidar_location_set(:,1),kf_lidar_location_set(:,2),'g.');
scatter(true_location_set(:,1),true_location_set(:,2),'k.');
legend('lidar','radar','lidar-kf','groudtruth')
hold off;
xlim([0,120])
ylim([40,180])

subplot(1,4,2)
scatter(lidar_location_set(:,1),lidar_location_set(:,2),'r.');
legend('lidar')
xlim([0,120])
ylim([40,180])

subplot(1,4,3)
scatter(radar_location_set(:,1),radar_location_set(:,2),'b.');
legend('radar')
xlim([0,120])
ylim([40,180])

subplot(1,4,4)
scatter(kf_lidar_location_set(:,1),kf_lidar_location_set(:,2),'g.');
legend('lidar-kf')
xlim([0,120])
ylim([40,180])
