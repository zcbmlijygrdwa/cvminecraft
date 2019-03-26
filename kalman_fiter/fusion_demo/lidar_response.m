function response = lidar_response(location,noise_param)

    %add noise
    location.x = location.x + normrnd(noise_param.mu,noise_param.sigma);
    location.y = location.y + normrnd(noise_param.mu,noise_param.sigma);
    
    response.location = location;
end