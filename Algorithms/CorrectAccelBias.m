% Calculate Average ACCEL BIAS
function [NewAccelData] = CorrectAccelBias(acceldata,stepper_knee_angle,gait_stage)
    % Init the vectors you'd like to center about (in G's). [X,Y,Z]
    % These were collected from ImageJ software to confirm location.
    % (Gx1 = sin26.5, Gy1 = -cos26.5; Gx2 = sin6.2, Gy2 = -cos6.2)
    A1_BIAS = [0.446,-0.895,0.000]'; A2_BIAS = [0.108,-0.994,0.000]';

    % Init
    ax_1_ZERO_ = 0; ay_1_ZERO_ = 0; az_1_ZERO_ = 0; 
    ax_2_ZERO_ = 0; ay_2_ZERO_ = 0; az_2_ZERO_= 0;
    count = 0;
    
    % Collect average zeroing
    for i = 1:length(acceldata(1,:))
        if ((stepper_knee_angle(i) == 4.06) && (gait_stage(i) == 8))
            ax_1_ZERO_ = acceldata(1,i) + ax_1_ZERO_;
            ay_1_ZERO_ = acceldata(2,i) + ay_1_ZERO_;
            az_1_ZERO_ = acceldata(3,i) + az_1_ZERO_;

            ax_2_ZERO_ = acceldata(4,i) + ax_2_ZERO_;
            ay_2_ZERO_ = acceldata(5,i) + ay_2_ZERO_;
            az_2_ZERO_ = acceldata(6,i) + az_2_ZERO_;

            count = count + 1;
        end
    end

    % Average bias
    ax_1_ZERO = ax_1_ZERO_ / count;
    ay_1_ZERO = ay_1_ZERO_ / count;
    az_1_ZERO = az_1_ZERO_ / count;
    
    ax_2_ZERO = ax_2_ZERO_ / count;
    ay_2_ZERO = ay_2_ZERO_ / count;
    az_2_ZERO = az_2_ZERO_ / count;
    
    % Init
    NewAccelData = zeros(size(acceldata));
    
    % Zeroing the accels and reflect
    for j = 1:length(acceldata(1,:))
        NewAccelData(1,j) = -(acceldata(1,j)-ax_1_ZERO);
        NewAccelData(2,j) = -(acceldata(2,j)-ay_1_ZERO);
        NewAccelData(3,j) = -(acceldata(3,j)-az_1_ZERO);

        NewAccelData(4,j) = -(acceldata(4,j)-ax_2_ZERO);
        NewAccelData(5,j) = -(acceldata(5,j)-ay_2_ZERO);
        NewAccelData(6,j) = -(acceldata(6,j)-az_2_ZERO);
    end
    % Biasing the accels
    NewAccelData(1,:) = NewAccelData(1,:) + A1_BIAS(1);
    NewAccelData(2,:) = NewAccelData(2,:) + A1_BIAS(2);
    NewAccelData(3,:) = NewAccelData(3,:) + A1_BIAS(3);

    NewAccelData(4,:) = NewAccelData(4,:) + A2_BIAS(1);
    NewAccelData(5,:) = NewAccelData(5,:) + A2_BIAS(2);
    NewAccelData(6,:) = NewAccelData(6,:) + A2_BIAS(3);
end
