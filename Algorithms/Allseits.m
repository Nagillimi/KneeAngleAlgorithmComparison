function knee_angle_Allseits = Allseits(f,gx_1,gy_1,gz_1,gx_2,gy_2,gz_2)
    % Collect individual datasets
    gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
    
    % Length of dataset
    N = length(gx_1);
    
    % Obtaining j vectors, following Seel's 2014 study
    [j1,j2,~] = localJ(gyrodata,0);
    
    % Convert gyros to JCS (using j's).
    % c can't be parallel to either j-vector
    c = [0,1,0]'; 
    
    % Define joint axes orthogonal to joint vector
    x1 = cross(j1,c); y1 = cross(j1,x1);
    x2 = cross(j2,c); y2 = cross(j2,x2);
    
    % Init
    delta_g = zeros(3,N); g1_JCS = zeros(3,N); g2_JCS = zeros(3,N);
    
    % Populate gyro data to JCS components
    for i = 1:N
        g1_JCS(1:3,i) = [dot(gyrodata(1:3,i),x1),dot(gyrodata(1:3,i),y1),dot(gyrodata(1:3,i),j1)]';
        g2_JCS(1:3,i) = [dot(gyrodata(4:6,i),x2),dot(gyrodata(4:6,i),y2),dot(gyrodata(4:6,i),j2)]';

        delta_g(1:3,i) = [g1_JCS(1,i)-g2_JCS(1,i),g1_JCS(2,i)-g2_JCS(2,i),g1_JCS(3,i)-g2_JCS(3,i)]';
    end
    
    % LP filter
    [Bl,Al] = butter(2,1.5/100,'low');
    delta_g(3,:) = filtfilt(Bl,Al,delta_g(3,:));
    g2_JCS(3,:) = filtfilt(Bl,Al,g2_JCS(3,:));
    
    % ZKA Algorithm to find the ZKA indices from shank gyro data
    zero_index = ZKA(g2_JCS(3,:),8);
        
    % Integrate delta of FE data (Z) to get knee angle
    t = (1/f).*(1:1:N)'; 
    knee_angle_Allseits = cumtrapz(t, delta_g(3,:));
    
    % Apply the ZKA indices to correct drift
    for i = 1:8
        zero_bias = knee_angle_Allseits(zero_index(i)) - 0; % @ Zero angle
        knee_angle_Allseits(zero_index(i):end) = knee_angle_Allseits(zero_index(i):end) - zero_bias;
        
%         hs_bias = knee_angle_Allseits(hs_index(i)) - 4; % @ 4deg angle
%         knee_angle_Allseits(hs_index(i):zero_index(i+1)) = knee_angle_Allseits(hs_index(i):zero_index(i+1)) + hs_bias;
    end
    
    % Debugging Plots
%     figure(1)
%     t = zeros(1,N);
%     plot(g2_JCS(1,1:end))
%     hold on
%     plot(g2_JCS(2,1:end))
%     plot(g2_JCS(3,1:end))
%     plot(t(1:1000),'k:')
%     plot(stepper_knee_angle(1:end))
%     legend('x','y','z')
%     hold off
%     
%     figure(2)
%     plot(stepper_knee_angle)
%     hold on
%     plot(knee_angle_Allseits)
%     plot(t,'k:') 
%     hold off
    
end
