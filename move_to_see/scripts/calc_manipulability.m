function [manipulability] = calc_manipulability(q_cur, robotype)
    manipulability = zeros(9,1);
    
    % Create robot model
    if robotype == 1
        % Create UR5 model
        robot = mdl_ur5_harvey();
        
        % Transformation matrix of reference camera wrt end-effector
        T_ref_ee = [1.0, 0.0, 0.0, 0.0
                    0.0, 1.0, 0.0, 0.0
                    0.0, 0.0, 1.0, 0.0
                    0.0, 0.0, 0.0, 1.0];
    end
    
    % Transformation matrices between all cameras wrt reference camera
    T_rel = zeros(4,4,9);
    T_rel(:,:,1) = [1.0, 0.0, 0.0, 0.030310906
                    0.0, 1.0, 0.0, 0.035000085
                    0.0, 0.0, 1.0, 0.017500072
                    0.0, 0.0, 0.0, 1.0];
    T_rel(:,:,2) = [1.0, 0.0, 0.0, 7.450580596e-9
                    0.0, 1.0, 0.0, 0.034999847
                    0.0, 0.0, 1.0, 0.009378194
                    0.0, 0.0, 0.0, 1.0];  
    T_rel(:,:,3) = [1.0, 0.0, 0.0, -0.03031075
                    0.0, 1.0, 0.0, 0.035000205
                    0.0, 0.0, 1.0, 0.017499834
                    0.0, 0.0, 0.0, 1.0];   
    T_rel(:,:,4) = [1.0, 0.0, 0.0, 0.034999951
                    0.0, 1.0, 0.0, -1.19209289e-7
                    0.0, 0.0, 1.0, 0.009378224
                    0.0, 0.0, 0.0, 1.0];
    T_rel(:,:,5) = eye(4);
    T_rel(:,:,6) = [1.0, 0.0, 0.0, -0.03499995
                    0.0, 1.0, 0.0, 0.0
                    0.0, 0.0, 1.0, 0.009378314
                    0.0, 0.0, 0.0, 1.0];  
    T_rel(:,:,7) = [1.0, 0.0, 0.0, 0.030311025
                    0.0, 1.0, 0.0, -0.03499996
                    0.0, 0.0, 1.0, 0.017499834
                    0.0, 0.0, 0.0, 1.0];  
    T_rel(:,:,8) = [1.0, 0.0, 0.0, -1.49011611e-8
                    0.0, 1.0, 0.0, -0.03499984
                    0.0, 0.0, 1.0, 0.009378194
                    0.0, 0.0, 0.0, 1.0];  
    T_rel(:,:,9) = [1.0, 0.0, 0.0, -0.03031078
                    0.0, 1.0, 0.0, -0.03500020
                    0.0, 0.0, 1.0, 0.017500072
                    0.0, 0.0, 0.0, 1.0];
    
    % FK to find transformation from base to reference camera
    T_ee  = fkine( robot, q_cur );      % end-effector
    T_ref = double( T_ee ) * T_ref_ee;	% reference camera
    assignin('caller', 'T_ref', T_ref);
    
    % Loop through all cameras
    for i = 1:9
        % Calculate transformation camera i wrt base
        T_tot = T_rel(:,:,i) * T_ref;
        
        % Do inverse kinematics to find joint values
        q = ikine( robot, T_tot );
        
        % Jacobian & manipulability cam i (in python idx-1)
        J0 = jacob0( robot, double(q) );
        manipulability(i) = sqrt( det(J0*J0') );
    end
    
%     % for debugging put in workspace
%     deg = pi/180;
%     assignin('caller', 'ur5_harvey', robot);
%     assignin('caller', 'qz', [0 0 0 0 0 0]); % zero angles
%     assignin('caller', 'qr', [180 0 0 0 90 0]*deg); % vertical pose as per Fig 2
end

