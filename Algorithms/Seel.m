function [acceldata,accel_angle,gyro_angle,knee_angle_Seel] = Seel(gx_1,gy_1,gz_1,...
    ax_1,ay_1,az_1,gx_2,gy_2,gz_2,ax_2,ay_2,az_2,stepper_knee_angle,gait_stage)
    % As defined in (Seel, 2014), can range from [0,1]. 
    % The sensor fusion constant
    lambda = 0.001; 
    
    % Vector maintains a non-zero product with j1 and j2 (can't be parallel
    % to either j1 or j2!). Init as [1,0,0]' as mentioned in (Seel, 2014).
    c = [1,0,0]';
    
    % Collect individual datasets
    gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
    acceldata_ = [ax_1,ay_1,az_1,ax_2,ay_2,az_2]';
    
    % Correct acceldata (from calibration error)
    acceldata = CorrectAccelBias(acceldata_,stepper_knee_angle,gait_stage);
    
    % Length of dataset
    N = length(gyrodata(1,:));
    
    % Obtaining j vectors, following Seel's 2014 study
    [j1,j2,~] = localJ(gyrodata,true);
    
    % Obtaining o-vectors, following Seel's 2012/2014 study 
    [~,~,gyro_dotdata,~] = localO(acceldata,gyrodata,true);
    
    %====================================================================
    % Overwrite o-vectors for testing, measured from ImageJ.
    o1 = [0.644,-0.715,0]';
    o2 = [0.063,0.998,0]';
    %====================================================================
    
    % Shift o-vectors to be closer to sensors. "The final step in of an 
    % automatic algorithm for joint position identification"
    o1 = o1 - j1*(dot(o1,j1) + dot(o2,j2))/2;
    o2 = o2 - j2*(dot(o1,j1) + dot(o2,j2))/2;
    
    % Gyro-derived angle
    gyro_angle_ = zeros(N,1); gyro_angle = zeros(N,1);
    for i = 1:N
        % Integrate
        gyro_angle_(i) = dot(gyrodata(1:3,i),j1) - dot(gyrodata(4:6,i),j2);
    end
    % Populate integral
    for i = 2:N-1
        gyro_angle(i-1) = mean(gyro_angle_(i-1:i+1)).*2;
    end
        
    % Define joint axes for accel data
    x1 = cross(j1,c); y1 = cross(j1,x1);
    x2 = cross(j2,c); y2 = cross(j2,x2);
    
    % Accel-based quantities
    a1_lambda = zeros(3,N); a2_lambda = zeros(3,N);
    u = zeros(2,N); v = zeros(2,N); accel_angle = zeros(N,1);
    
    % Isolate accelerations from tangential accels
    for i = 1:N
        rot1 = cross(gyrodata(1:3,i),cross(gyrodata(1:3,i),o1))...
                + cross(gyro_dotdata(1:3,i),o1);
        rot2 = cross(gyrodata(4:6,i),cross(gyrodata(4:6,i),o2))...
                + cross(gyro_dotdata(4:6,i),o2);
        
        % Isolating the equal parts of the accels from the rotation
        a1_lambda(1:3,i) = acceldata(1:3,i) - rot1;
        a2_lambda(1:3,i) = acceldata(4:6,i) - rot2;
        
        % Constructing 2d accel vectors in joint frame
        u(1:2,i) = [dot(a1_lambda(:,i),x1),dot(a1_lambda(:,i),y1)]';
        v(1:2,i) = [dot(a2_lambda(:,i),x2),dot(a2_lambda(:,i),y2)]';
    end
    
    % Accel-derived angle
    for i = 1:N
        % Accel-derived angle
        
        % Complex plane solution
%         x = dot(u(:,i), [1, 1i]');
%         y = dot(v(:,i), [1, 1i]');
%         accel_angle(i) = angle(x*y');
        
        % Other vectorized solution
%         accel_angle(i) = atan2d(u(:,i) * (v([2,1],i)'.*[1 -1]).',u(:,i) * v(:,i).');
        
        % Normalized vector dot Product
%         uN = u(:,i)./norm(u(:,i)); vN = v(:,i)./norm(v(:,i));
%         accel_angle(i) = acosd(dot(uN,vN))./5;
        
        % Definition of Dot Product
%         accel_angle(i) = acosd((dot(v(1:2,i),u(1:2,i)))...
%             /(norm(v(1:2,i))*norm(u(1:2,i))));
        
        % Definition of Cross Product
%         accel_angle(i) = asind( (u(1,i)*v(2,i) - u(2,i)*v(1,i))...
%             ./(norm(u(:,i))*norm(v(:,i))) );
        
%         accel_angle(i) = acosd(min(1,max(-1, u(:,i).'*v(:,i) / norm(u(:,i)) / norm(v(:,i)) )));
        

%         CosTheta = max(min(dot(u(:,i),v(:,i))/(norm(u(:,i))*norm(v(:,i))),1),-1);
%         accel_angle(i) = real(acosd(CosTheta));
        

        % https://www.mathworks.com/matlabcentral/answers/180131-how-can-i-find-the-angle-between-two-vectors-including-directional-information
        accel_angle(i) = atan2d(u(1,i)*v(2,i) - u(2,i)*v(1,i),...
                                u(1,i)*v(1,i) + u(2,i)*v(2,i));
    end
    
    % Sensor fusion (Seel, 2014)
    knee_angle_Seel = zeros(N,1);
    for i = 2:N
        knee_angle_Seel(i) = lambda*accel_angle(i)...
            + (1-lambda)*(knee_angle_Seel(i-1) + gyro_angle(i) - gyro_angle(i-1));
        
    end
    
    % Low Pass filter
    [Bl,Al] = butter(2,1.5/100,'low');
    knee_angle_Seel = filtfilt(Bl,Al,knee_angle_Seel);
    gyro_angle = filtfilt(Bl,Al,gyro_angle);
    accel_angle = filtfilt(Bl,Al,accel_angle);
end
