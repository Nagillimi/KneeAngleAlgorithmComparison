function [acceldata,accel_angle,gyro_angle,knee_angle_Seel] = Seel(f,gx_1,gy_1,gz_1,...
    ax_1,ay_1,az_1,gx_2,gy_2,gz_2,ax_2,ay_2,az_2,stepper_knee_angle,gait_stage)
    % As defined in (Seel, 2014), can range from [0,1]. 
    % The sensor fusion constant, he did 0.01
    lambda = 0.01; 
    
    % Vector maintains a non-zero product with j1 and j2 (can't be parallel
    % to either j1 or j2!). Init as [1,0,0]' as mentioned in (Seel, 2014).
    c = [0,1,0]';
    
    % Collect individual datasets
    gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
    acceldata_ = [ax_1,ay_1,az_1,ax_2,ay_2,az_2]';
    
    % Correct acceldata (from calibration error)
    acceldata = CorrectAccelBias(acceldata_,stepper_knee_angle,gait_stage);
%     acceldata = acceldata_; % Or don't correct it
    
    % Length of trial dataset
    N = length(gx_1);
    
    % Obtaining j vectors, following Seel's 2014 study
    [j1,j2,~] = localJ(gyrodata,0);
    
    % Obtaining o-vectors, following Seel's 2012/2014 study 
    [o1,o2,gyro_dotdata,~] = localO(acceldata,gyrodata,0);
    
    %====================================================================
    % Overwrite o-vectors for testing, approx using ImageJ.
%     o1 = [0.644,-0.715,0]';
%     o2 = [0.063,0.998,0]';
    %====================================================================
    
    % Shift o-vectors to be closer to sensors. "The final step in of an 
    % automatic algorithm for joint position identification"
    o1 = o1 - j1*(dot(o1,j1) + dot(o2,j2))/2;
    o2 = o2 - j2*(dot(o1,j1) + dot(o2,j2))/2;
    
    %---------------------------GYRO-------------------------------------
    % Gyro-derived angle
    gyro_angle_ = zeros(N,1); 
    % Time Vector
    t = (1/f).*(1:1:N)'; 
    for i = 1:N
        % Integrate
        gyro_angle_(i) = dot(gyrodata(1:3,i),j1) - dot(gyrodata(4:6,i),j2);
    end
    % Populate integral with cumtrapz()
    gyro_angle = cumtrapz(t, gyro_angle_);
        
    %---------------------------ACCEL------------------------------------
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
        u(1:2,i) = [(dot(a1_lambda(:,i),x1)),(dot(a1_lambda(:,i),y1))]';
        v(1:2,i) = [(dot(a2_lambda(:,i),x2)),(dot(a2_lambda(:,i),y2))]';
    end
    
    % Accel-derived angle
    alpha = zeros(N,1); beta = zeros(N,1);
    for i = 1:N
        % Accel-derived angle
        
        % Double Angle solution. [atan(y/x)]
        alpha(i) = abs(atand(u(2,i)/u(1,i))); beta(i) = abs(atand(v(1,i)/v(2,i)));
        
        % Various angle conditions
        
%         % Opposite ends of x-coord OR y-coord
%         if (u(1,i) > 0 && v(1,i) < 0) || (u(2,i) > 0 && v(2,i) < 0) || ...
%            (v(1,i) > 0 && u(1,i) < 0) || (v(2,i) > 0 && u(2,i) < 0)
%             accel_angle(i) = beta(i) + alpha(i);            
% 
%         % Same grid of x-coord OR y-coord
%         elseif (u(1,i) > 0 && v(1,i) > 0 && u(2,i) > 0 && v(2,i) > 0) || ...
%                (u(1,i) < 0 && v(1,i) < 0 && u(2,i) < 0 && v(2,i) < 0) || ...
%                (u(1,i) > 0 && v(1,i) > 0 && u(2,i) < 0 && v(2,i) < 0) || ...
%                (u(1,i) < 0 && v(1,i) < 0 && u(2,i) > 0 && v(2,i) > 0)
%             accel_angle(i) = (beta(i) - alpha(i));
%             
%         % Diagonally opposite grid of x-coord OR y-coord
%         elseif (u(1,i) > 0 && v(1,i) < 0 && u(2,i) > 0 && v(2,i) < 0) || ...
%                (u(1,i) < 0 && v(1,i) > 0 && u(2,i) < 0 && v(2,i) > 0)
%             accel_angle(i) = (beta(i) - alpha(i));
%         end
        
        accel_angle(i) = abs(90 - (beta(i) + alpha(i)));
%         % u on RS, v on LS
%         if (u(1,i)>0) && (u(2,i)>0) && (v(1,i)<0) && (v(2,i)>0)
%             accel_angle(i) = alpha(i) + beta(i);
%         elseif (u(1,i)>0) && (u(2,i)>0) && (v(1,i)<0) && (v(2,i)<0)
%             accel_angle(i) = alpha(i) + beta(i);
%         elseif (u(1,i)>0) && (u(2,i)<0) && (v(1,i)<0) && (v(2,i)>0)
%             accel_angle(i) = alpha(i) + beta(i);
%         elseif (u(1,i)>0) && (u(2,i)<0) && (v(1,i)<0) && (v(2,i)<0)
%             accel_angle(i) = alpha(i) + beta(i);
%             
%         % u on LS, v on RS
%         elseif (v(1,i)>0) && (v(2,i)>0) && (u(1,i)<0) && (u(2,i)>0)
%             accel_angle(i) = alpha(i) + beta(i);
%         elseif (v(1,i)>0) && (v(2,i)>0) && (u(1,i)<0) && (u(2,i)<0)
%             accel_angle(i) = alpha(i) + beta(i);
%         elseif (v(1,i)>0) && (v(2,i)<0) && (u(1,i)<0) && (u(2,i)>0)
%             accel_angle(i) = alpha(i) + beta(i);
%         elseif (v(1,i)>0) && (v(2,i)<0) && (u(1,i)<0) && (u(2,i)<0)
%             accel_angle(i) = alpha(i) + beta(i);
%         
%         % u on RS, v on RS
%         elseif (u(1,i)>0) && (u(2,i)>0) && (v(1,i)>0) && (v(2,i)>0)
%             accel_angle(i) = beta(i) - alpha(i);
%         elseif (u(1,i)>0) && (u(2,i)<0) && (v(1,i)>0) && (v(2,i)>0)
%             accel_angle(i) = beta(i) - alpha(i);
%         elseif (u(1,i)>0) && (u(2,i)<0) && (v(1,i)>0) && (v(2,i)<0)
%             accel_angle(i) = beta(i) - alpha(i);
%         elseif (u(1,i)>0) && (u(2,i)>0) && (v(1,i)>0) && (v(2,i)<0)
%             accel_angle(i) = beta(i) - alpha(i);
%         
%         % u on LS, v on LS
%         elseif (v(1,i)>0) && (v(2,i)>0) && (u(1,i)>0) && (u(2,i)>0)
%             accel_angle(i) = beta(i) - alpha(i);
%         elseif (v(1,i)>0) && (v(2,i)<0) && (u(1,i)>0) && (u(2,i)>0)
%             accel_angle(i) = beta(i) - alpha(i);
%         elseif (v(1,i)>0) && (v(2,i)<0) && (u(1,i)>0) && (u(2,i)<0)
%             accel_angle(i) = beta(i) - alpha(i);
%         elseif (v(1,i)>0) && (v(2,i)>0) && (u(1,i)>0) && (u(2,i)<0)
%             accel_angle(i) = beta(i) - alpha(i);
%         end
        
    end
    
    %---------------------------COMBINE----------------------------------    
    % Sensor fusion (Seel, 2014)
    knee_angle_Seel = zeros(N,1);
    for i = 2:N
        knee_angle_Seel(i) = lambda*accel_angle(i)...
            + (1-lambda)*(knee_angle_Seel(i-1) + gyro_angle(i) - gyro_angle(i-1));
        
    end
    
    % Low Pass filter
    [Bl,Al] = butter(2,2/100,'low');
    knee_angle_Seel = filtfilt(Bl,Al,knee_angle_Seel);
    gyro_angle = filtfilt(Bl,Al,gyro_angle);
    accel_angle = filtfilt(Bl,Al,accel_angle);
%     u(1,:) = filtfilt(Bl,Al,u(1,:));
%     v(1,:) = filtfilt(Bl,Al,v(1,:));
%     u(2,:) = filtfilt(Bl,Al,u(2,:));
%     v(2,:) = filtfilt(Bl,Al,v(2,:));
%     alpha = filtfilt(Bl,Al,alpha);
%     beta = filtfilt(Bl,Al,beta);
end


% figure(2)
% plot(u(1,:))
% hold on
% plot(v(1,:))
% title('X-Coordinate JCS')
% 
% figure(3)
% plot(u(2,:))
% hold on
% plot(v(2,:))
% title('Y-Coordinate JCS')
% 
% figure(4)
% subplot(2,1,1)
% plot(v(1,:)-u(1,:))
% title('Difference between X-Coordinates')
% subplot(2,1,2)
% plot(v(2,:)-u(2,:))
% title('Difference between Y-Coordinates')
% 
% figure(5)
% plot(accel_angle(1:5000))
% hold on
% plot(gyro_angle(1:5000))
% plot(knee_angle_Seel(1:5000))
% 
% figure(6)
% plot(alpha)
% hold on
% plot(beta)
% legend('alpha','beta')
