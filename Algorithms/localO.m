function [o1,o2,gyro_dotdata] = localO(acceldata,gyrodata,displaygraph)

    tol = 0.0001; N = length(gyrodata(1,:)); axislength = 30;

    % Initial guess
    x(:,1) = [rand*pi,rand*pi,rand*pi,rand*pi]';
    
    % Calculate gyroscope derivatives
    gyro_dotdata = g_deriv(gyrodata);
    
    i = 1; v = 1;
    while v == 1
        % Calculate o-vectors and error vector. Step 1 & 2.
        e = e_vector(x(:,i),acceldata,gyrodata,gyro_dotdata);
        
        % Calculate Jacobian. Step 3.
        de_dx = polar_gradient(x(:,i),acceldata,gyrodata,gyro_dotdata);
        
        % Gauss-Newton step. Step 4.
        x(:,i+1) = x(:,i) - pinv(de_dx)*e;
        
        % Converged
        if (x(:,i+1)-x(:,i) < tol)
            v = 0;
            % Flatline x vector from converged value
            for k = i:N
                x(:,k) = x(:,i+1);
            end
            
            % Return o-vectors from final value (ie, the flatline)
            o1 = -RM_spher(x(1,end),x(2,end))*[1;0;0];
            o2 = -RM_spher(x(3,end),x(4,end))*[1;0;0];
            
            % Confirm o-vector signs
            
            
            disp('O-vectors converged at iteration: ')
            disp(i+1)
            disp('o1 = ')
            disp(o1)
            disp('o2 = ')
            disp(o2)
        elseif i == N
            v = 0;
            disp('Error: Tolerance too tight')
        end
        
        i = i + 1;
    end
    
    if displaygraph == true
        % Figures for debugging
        figure(2)
        plot(x(1,:))
        hold on
        plot(x(2,:))
        plot(x(3,:))
        plot(x(4,:))
        title('O-vectors in Polar Coordinates');
        ylabel('Radians');
        xlabel('Gradient-Descent Iteration');
        legend({'\theta_1','\phi_1','\theta_2','\phi_2'});
        xlim([1,axislength]);
        hold off
    end
end

% Helper functions
function M = RM_spher(a,i)
    % Trig substitutes
    ca = cos(a); ci = cos(i); sa = sin(a); si = sin(i);
    
    % Return M
    M = [ca*ci, -sa, -ca*si;
         sa*ci, ca, -sa*si;
         si, 0, ci];
end

function g_dot = g_deriv(gyrodata)
    % Init g_dot to the same size as gyrodata
    g_dot = zeros(size(gyrodata));
    
    for i = 3:(length(gyrodata(1,:)) - 2)
        % Estimate gyroscope derivative with 3rd order approx
        g_dot(1:3,i) = ( gyrodata(1:3,i-2) - 8*(gyrodata(1:3,i-1))...
            + 8*(gyrodata(1:3,i+1)) - gyrodata(1:3,i+2) ) ./ 12;
        g_dot(4:6,i) = ( gyrodata(4:6,i-2) - 8*(gyrodata(4:6,i-1))...
            + 8*(gyrodata(4:6,i+1)) - gyrodata(4:6,i+2) ) ./ 12;
    end
end

function e_vec = e_vector(x,acceldata,gyrodata,gyro_dotdata)
    % Ignore NaN col's
    gyrodata = gyrodata(:,~isnan(gyrodata(1,:)));
    acceldata = acceldata(:,~isnan(acceldata(1,:)));
    
    % Get o-vector estimates at index k (from main)
    o_k_us_est = RM_spher(x(1),x(2))*[1;0;0];
    o_k_ls_est = RM_spher(x(3),x(4))*[1;0;0];
    
    % Compute error vector
    e_vec = ones(length(gyrodata(1,:)),1);
    for i = 1:length(gyrodata(1,:))
        temp1 = cross(gyrodata(1:3,i),cross(gyrodata(1:3,i),o_k_us_est))...
                + cross(gyro_dotdata(1:3,i),o_k_us_est);
        temp2 = cross(gyrodata(4:6,i),cross(gyrodata(4:6,i),o_k_ls_est))...
                + cross(gyro_dotdata(4:6,i),o_k_ls_est);
        
        e_vec(i) = 1.*(norm(acceldata(1:3,i) - temp1) - norm(acceldata(4:6,i) - temp2));
    end
end

function grad = polar_gradient(x,acceldata,gyrodata,gyro_dotdata)
    % Ignore NaN col's
    gyrodata = gyrodata(:,~isnan(gyrodata(1,:)));
    acceldata = acceldata(:,~isnan(acceldata(1,:)));
    
    % Get o-vector estimates at index k (from main)
    o_k_us_est = RM_spher(x(1),x(2))*[1;0;0];
    o_k_ls_est = RM_spher(x(3),x(4))*[1;0;0];
    
    % Compute Jacobian
    de_do = ones(length(gyrodata(1,:)),6);
    for i = 1:length(de_do(:,1))
        temp1 = cross(gyrodata(1:3,i),cross(gyrodata(1:3,i),o_k_us_est))...
                + cross(gyro_dotdata(1:3,i),o_k_us_est);
        temp2 = cross(gyrodata(4:6,i),cross(gyrodata(4:6,i),o_k_ls_est))...
                + cross(gyro_dotdata(4:6,i),o_k_ls_est);
            
        oo_1 = acceldata(1:3,i) - temp1;
        oo_2 = acceldata(4:6,i) - temp2;
        
        num1 = cross(cross(oo_1,gyrodata(1:3,i)),gyrodata(1:3,i))...
                + cross(oo_1,gyro_dotdata(1:3,i));
        num2 = cross(cross(oo_2,gyrodata(4:6,i)),gyrodata(4:6,i))...
                + cross(oo_2,gyro_dotdata(4:6,i));
        
        de_do(i,:) = 1.*[num1/norm(oo_1);
                        -num2/norm(oo_2)]';
    end
    
    % Trig substitutes
    s1 = sin(x(1)); s2 = sin(x(2)); c1 = cos(x(1)); c2 = cos(x(2));
    s3 = sin(x(3)); s4 = sin(x(4)); c3 = cos(x(3)); c4 = cos(x(4));
    
    do_dx= [-s1*c2, -c1*s2, 0, 0;
            c1*c2, -s1*s2, 0, 0;
            0, c2, 0, 0;
            0, 0, -s3*c4, -c3*s4;
            0, 0, c3*c4, -s3*s4;
            0, 0, 0, c4];
    
    % Return gradient
    grad = de_do*do_dx;
end