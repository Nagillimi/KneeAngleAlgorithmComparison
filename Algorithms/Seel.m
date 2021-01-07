function knee_angle_Seel = Seel(gx_1,gy_1,gz_1,ax_1,ay_1,az_1,gx_2,gy_2,gz_2,ax_2,ay_2,az_2)
    % As defined in (Seel, 2014), can range from [0,1]. 
    % The sensor fusion constant
    lambda = 0.01; 
    
    % Vector maintains a non-zero product with j1 and j2 (can't be parallel
    % to either j1 or j2!). Init as [1,0,0]' as mentioned in (Seel, 2014).
    c = [1,0,0]';
    
    % Collect individual datasets
    gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
    acceldata = [ax_1,ay_1,az_1,ax_2,ay_2,az_2]';
    
    % Obtaining j vectors, following Seel's 2014 study
    [j1,j2] = localJ(gyrodata,true);
    
    % Obtaining o-vectors, following Seel's 2012/2014 study 
    [o1,o2,gyro_dotdata] = localO(acceldata,gyrodata,true);
    
    % Gyro-derived angle
    gyro_angle = zeros(length(gx_1),1);
    for i = 1:length(gyrodata(1,:))
        gyro_angle(i) = dot(gyrodata(1:3,i),j1) - dot(gyrodata(4:6,i),j2);
    end
        
    % Define joint axes for accel data
    x1 = cross(j1,c); y1 = cross(j1,x1);
    x2 = cross(j2,c); y2 = cross(j2,x2);
    
    % Accel-based quantities
    a1 = zeros(length(gx_1),1); a2 = zeros(length(gx_1),1);
    a1_t = zeros(length(gx_1),1); a2_t = zeros(length(gx_1),1);
    accel_angle = zeros(length(gx_1),1);
    for i = 1:length(gyrodata(1,:))
        temp1 = cross(gyrodata(1:3,i),cross(gyrodata(1:3,i),o1))...
                + cross(gyro_dotdata(1:3,i),o1);
        temp2 = cross(gyrodata(4:6,i),cross(gyrodata(4:6,i),o2))...
                + cross(gyro_dotdata(4:6,i),o2);
        
        a1(1:3,i) = acceldata(1:3,i) - temp1;
        a2(1:3,i) = acceldata(4:6,i) - temp2;
        
        % Constructing 2d accel vectors
        a1_t(1:2,i) = [dot(a1(1:3,i),x1),dot(a1(1:3,i),y1)]';
        a2_t(1:2,i) = [dot(a2(1:3,i),x2),dot(a2(1:3,i),y2)]';
        
        % Accel-derived angle    
        accel_angle(i) = atan2d(a1_t(1,i)*a2_t(2,i) - a1_t(2,i)*a2_t(1,i),...
                                a1_t(1,i)*a2_t(1,i) - a1_t(2,i)*a2_t(2,i));
    end
    
    % Sensor fusion
    knee_angle_Seel = zeros(length(gx_1),1);
    for i = 2:length(gx_1)
        knee_angle_Seel(i) = lambda*accel_angle(i)...
            + (1-lambda)*(knee_angle_Seel(i-1) + gyro_angle(i) - gyro_angle(i-1));
        
    end
end
