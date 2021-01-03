function [knee_angle_Allseits,error_Allseits] = Allseits(gx_1,gy_1,gz_1,gx_2,gy_2,gz_2)
    
    gyrodata = [gx_1,gy_1,gz_1,gx_2,gy_2,gz_2]';
    
    % Obtaining j vectors, following Seel's 2014 study
    [j1,j2] = localJ(gyrodata,true);
    J = [j1,j2];

    % Converting g's to global CS. Follow a Cardan Rotation Sequence = XZ. 
    % Puts y parallel to joint vector
    for i = 1:2
        % Test the signs of these rotations...
        a = atan(J(3,i)); b = atan(J(1,i)/sqrt(J(3,i)^2+J(2,i)^2));
        R = [1, 0, 0;
            0, cos(a), -sin(a);
            0, sin(a), cos(a)] * [cos(b), 0, sin(b);
                                  0, 1, 0;
                                  -sin(b), 0, cos(b)];
        % (#sensor,rows,cols)
        disp('R = ')
        disp(R)
%         G(i,3,:) = R*[gx_1,gy_1,gz_1]';
    end
    
    
    
   
    
    % 
     knee_angle_Allseits = 1;
    error_Allseits = 1;
end
