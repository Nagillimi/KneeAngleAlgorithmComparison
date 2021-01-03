function [knee_angle_Seel,error_Seel] = Seel(gx_1,gy_1,gz_1,ax_1,ay_1,az_1,gx_2,gy_2,gz_2,ax_2,ay_2,az_2)
    
    gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
    
    % Obtaining j vectors, following Seel's 2014 study
    [j1,j2] = localJ(gyrodata,true);
    
    % 
    
    % 
    
    
end