function knee_angle_Allseits = Allseits(f,gx_1,gy_1,gz_1,gx_2,gy_2,gz_2)
%     f = 100;
    
    % Remove NaN data
%     gx_1 = gx_1( ~any( isnan( gx_1 ) | isinf( gx_1 ), 2 ))
%     gy_1 = gy_1( ~any( isnan( gy_1 ) | isinf( gy_1 ), 2 ))
%     gz_1 = gz_1( ~any( isnan( gz_1 ) | isinf( gz_1 ), 2 ))
%     gx_2 = gx_2( ~any( isnan( gx_2 ) | isinf( gx_2 ), 2 ))
%     gy_2 = gy_2( ~any( isnan( gy_2 ) | isinf( gy_2 ), 2 ))
%     gz_2 = gz_2( ~any( isnan( gz_2 ) | isinf( gz_2 ), 2 ))
    
%     gx_1 = zero_order_hold_remove_nan(gx_1);
%     gy_1 = zero_order_hold_remove_nan(gy_1);
%     gz_1 = zero_order_hold_remove_nan(gz_1);
%     gx_2 = zero_order_hold_remove_nan(gx_2);
%     gy_2 = zero_order_hold_remove_nan(gy_2);
%     gz_2 = zero_order_hold_remove_nan(gz_2);
    
    
%     gx_1 = rmmissing(gx_1);
%     gy_1 = rmmissing(gy_1);
%     gz_1 = rmmissing(gz_1);
%     gx_2 = rmmissing(gx_2);
%     gy_2 = rmmissing(gy_2);
%     gz_2 = rmmissing(gz_2);
    
    % Resize if NaN found
%     data_no_NaN = [length(gx_1),length(gy_1),length(gz_1),length(gx_2),length(gy_2),length(gz_2)];
%     min_len = min(data_no_NaN);
%     gx_1 = gx_1(1:min_len);
%     gy_1 = gy_1(1:min_len);
%     gz_1 = gz_1(1:min_len);
%     gx_2 = gx_2(1:min_len);
%     gy_2 = gy_2(1:min_len);
%     gz_2 = gz_2(1:min_len);
    
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
