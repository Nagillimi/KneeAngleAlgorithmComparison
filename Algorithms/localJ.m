% Steepest descent algorithm tips:
% https://www.kdnuggets.com/2020/05/5-concepts-gradient-descent-cost-function.html

% Thomas said his j-vectors STATICALLY came to:
% j_upper = [-0.1,0.5,-0.9];
% j_lower = [0,1,0.1];
function [j1,j2,x] = localJ(gyrodata,displaygraph)
    tol = 0.0001; N = length(gyrodata(1,:)); axislength = 20;

    % Initial guess
    x(:,1) = [rand,rand,rand,rand]';
    
    i = 1; v = 1;
    while (v == 1) && (abs(x(1,i)) < 2*pi) && (abs(x(2,i)) < 2*pi)...
            && (abs(x(3,i)) < 2*pi) && (abs(x(4,i)) < 2*pi)
        % Calculate j-vectors and error vector. Step 1 & 2.
        e = e_vector(x(:,i),gyrodata);
        
        % Calculate Jacobian. Step 3.
        de_dx = polar_gradient(x(:,i),gyrodata);
        
        % Gauss-Newton step. Step 4.
        x(:,i+1) = x(:,i) - pinv(de_dx)*e;
        
        % Converged
        if (x(:,i+1)-x(:,i) < tol)
            v = 0;
            % Flatline x vector from converged value
            for k = i:N
                x(:,k) = x(:,i+1);
            end
            
            % Return j-vectors from final value (ie, the flatline)
            j1 = -RM_spher(x(1,end),x(2,end))*[1;0;0];
            j2 = -RM_spher(x(3,end),x(4,end))*[1;0;0];
            
            % Sign convention for j-vectors. Use the z-component (parallel 
            % to top face), should point medially.
            if j1(3) > 0
                j1 = -j1;
            end
            if j2(3) > 0
                j2 = -j2;
            end
            
            disp('J-vectors converged at iteration: ')
            disp(i+1)
            disp('j1 = ')
            disp(j1)
            disp('j2 = ')
            disp(j2)
        elseif i == N
            v = 0;
            disp('Error: Tolerance too tight')
        end
        % Iterate if no convergance
        i = i + 1;
    end
    
    % If angles exceeded 2pi and never converged
    if (v == 1)
        % Redo algorithm (recursive) without printing graphs
        % Overwrites the j1,j2,x variables
        clear j1 j2 x;
        [j1,j2,x] = localJ(gyrodata,0);
    end
    
    if displaygraph == true
        % Figures for debugging
        figure(1)
        subplot(2,1,1)
        plot(x(1,:))
        hold on
        plot(x(2,:))
        plot(x(3,:))
        plot(x(4,:))
        title('J-vectors in Polar Coordinates');
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

function e_vec = e_vector(x,gyrodata)
    gyrodata = gyrodata(:,~isnan(gyrodata(1,:))); % ignore nan-columns

    % Get j-vector estimates at index k (from main)
    j_k_us_est = RM_spher(x(1),x(2))*[1;0;0];
    j_k_ls_est = RM_spher(x(3),x(4))*[1;0;0];
    
    % Compute error vector
    e_vec = ones(length(gyrodata(1,:)),1);
    for i = 1:length(gyrodata(1,:))
        temp1 = cross(gyrodata(1:3,i),j_k_us_est);
        temp2 = cross(gyrodata(4:6,i),j_k_ls_est);
        
        e_vec(i) = 1.*(norm(temp1) - norm(temp2));
    end
end

function grad = polar_gradient(x,gyrodata)
    gyrodata = gyrodata(:,~isnan(gyrodata(1,:))); % ignore nan-columns
    
    % Get j-vector estimates at index k (from main)
    j_k_us_est = RM_spher(x(1),x(2))*[1;0;0];
    j_k_ls_est = RM_spher(x(3),x(4))*[1;0;0];
    
    % Compute Jacobian
    de_dj = ones(length(gyrodata(1,:)),6);
    for i = 1:length(de_dj(:,1))
        temp1 = cross(gyrodata(1:3,i),j_k_us_est);
        temp2 = cross(gyrodata(4:6,i),j_k_ls_est);
        
        de_dj(i,:) = 1.*[cross(temp1,gyrodata(1:3,i))/norm(temp1);
                        -cross(temp2,gyrodata(4:6,i))/norm(temp2)];
    end
    
    % Trig substitutes
    s1 = sin(x(1)); s2 = sin(x(2)); c1 = cos(x(1)); c2 = cos(x(2));
    s3 = sin(x(3)); s4 = sin(x(4)); c3 = cos(x(3)); c4 = cos(x(4));
    
    dj_dx= [-s1*c2, -c1*s2, 0, 0;
            c1*c2, -s1*s2, 0, 0;
            0, c2, 0, 0;
            0, 0, -s3*c4, -c3*s4;
            0, 0, c3*c4, -s3*s4;
            0, 0, 0, c4];
    
    % Return gradient
    grad = de_dj*dj_dx;
end
