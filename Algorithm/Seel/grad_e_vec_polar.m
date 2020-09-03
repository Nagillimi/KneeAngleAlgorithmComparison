% Gradient: de/dx, with x in polar coordinates
% x = (j1_theta, j1_phi, j2_theta, j2_phi)

function grad = grad_e_vec_polar(j,g1,g2) 
    % Compute the upper and lower j axis estimates
    % 3x3 by 3x1 = 3x1
    j1_est = ShperToRec(j(1),j(2))*[1;0;0];
    j2_est = ShperToRec(j(3),j(4))*[1;0;0];
    
    % Declare de/dj as a 38926x6 array.
    % gradient wrt [j_k_us;j_k_ls]
    de_dj=ones(length(transfer),6);
    
    for i=1:length(de_dj(:,1))
        
        % Cross 1x3 by 3x1 = 1x1 (scalar)
        % Temporary upper
        temp_1 = cross(g1(i,:),j1_est);
        % Temporary lower
        temp_2 = cross(g2(i,:),j2_est);
        
        % Compute de/dj
        de_dj(i,:)= 1.*[
            cross(temp_1,g1(:,i))/norm(temp_1)
            - cross(temp_2,g2(:,i))/norm(temp_2) ]';
    end
    
    % Assign trig variables to the joint shperical coord (just easier)
    s1 = sin(j(1)); s2 = sin(j(2)); c1 = cos(j(1)); c2 = cos(j(2));
    s3 = sin(j(3)); s4 = sin(j(4)); c3 = cos(j(3)); c4 = cos(j(4));
    
    % Init dj/dx derivative (6x4)
    dj_dx = [ -s1*c2, -c1*s2,    0,      0;
              c1*c2,  -s1*s2,    0,      0;
                 0,     c2,      0,      0;
                 0,      0,   -s3*c4, -c3*s4;
                 0,      0,    c3*c4, -s3*s4;
                 0,      0,      0,      c4  ];
    
    % Compute gradient, gives the direction of steepest descent
    % 38926x6 by 6x4 = 38926x4
    grad = de_dj*dj_dx;
end