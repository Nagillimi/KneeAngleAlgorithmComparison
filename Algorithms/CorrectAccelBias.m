% Calculate Average ACCEL BIAS
function [NewAccelData] = CorrectAccelBias(acceldata,stepper_knee_angle,impulse_hit,freq)
    % Init the vectors you'd like to center about (in G's). [X,Y,Z]
    % These were collected from ImageJ software to confirm location.
    ZeroAccel1 = [0,-1,0]; ZeroAccel2 = [0,-0.85,0];

    % Start of search begins 155s after beginning of trial (~around when 
    % final gait swing finishes)
    start = (155*freq);

    % Collect bias
    ax_1_BIAS_ = 0; ay_1_BIAS_ = 0; az_1_BIAS_ = 0; 
    ax_2_BIAS_ = 0; ay_2_BIAS_ = 0; az_2_BIAS_= 0;
    count = 0;
    for i = start:length(acceldata(1,:))
        if ((stepper_knee_angle(i) == 0) && (impulse_hit(i) == 0))
            ax_1_BIAS_ = acceldata(1,i) + ax_1_BIAS_;
            ay_1_BIAS_ = acceldata(2,i) + ay_1_BIAS_;
            az_1_BIAS_ = acceldata(3,i) + az_1_BIAS_;

            ax_2_BIAS_ = acceldata(4,i) + ax_2_BIAS_;
            ay_2_BIAS_ = acceldata(5,i) + ay_2_BIAS_;
            az_2_BIAS_ = acceldata(6,i) + az_2_BIAS_;

            count = count + 1;
        end
    end

    % Average bias
    ax_1_BIAS = ax_1_BIAS_ / count;
    ay_1_BIAS = ay_1_BIAS_ / count;
    az_1_BIAS = az_1_BIAS_ / count;
    
    ax_2_BIAS = ax_2_BIAS_ / count;
    ay_2_BIAS = ay_2_BIAS_ / count;
    az_2_BIAS = az_2_BIAS_ / count;
    
    % Init
    NewAccelData = zeros(size(acceldata));
    
    % Correcting the accels
    for j = 1:length(acceldata(1,:))
        NewAccelData(1,j) = -(acceldata(1,j)-ax_1_BIAS) + 0.45;
        NewAccelData(2,j) = -(acceldata(2,j)-ay_1_BIAS) - 0.89;
        NewAccelData(3,j) = -(acceldata(3,j)-az_1_BIAS);

        NewAccelData(4,j) = -(acceldata(4,j)-ax_2_BIAS) + 0.11;
        NewAccelData(5,j) = -(acceldata(5,j)-ay_2_BIAS) - 0.99;
        NewAccelData(6,j) = -(acceldata(6,j)-az_2_BIAS);
    end
end
