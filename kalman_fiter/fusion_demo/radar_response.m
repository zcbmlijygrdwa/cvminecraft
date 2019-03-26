function response = radar_response(location, location_prev, noise_param)

    %range rho: radial distance from origin
    rho = sqrt(location.x*location.x+location.y*location.y);
    
    %bearing phi: angle between rho and x
    phi = atan(location.y/location.x);
    
    
    rho_prev = sqrt(location_prev.x*location_prev.x+location_prev.y*location_prev.y);
    %radial velocity
    d_pho = rho - rho_prev;
    
    %add noise
    rho = rho + normrnd(noise_param.mu, noise_param.sigma);
    phi = phi + normrnd(noise_param.mu, noise_param.sigma);
    d_pho = d_pho + normrnd(noise_param.mu, noise_param.sigma);
    
    location.x = rho*cos(phi);
    location.y = rho*sin(phi);
    
    response.rho = rho;
    response.phi = phi;
    response.d_pho = d_pho;
    response.location = location;
    
    
end