% MDL_UR5_HARVEY is a script that creates the workspace variable ur5_harvey which
% describes the kinematic characteristics of a Universal Robotics UR5 manipulator
% as mounted on Harvey using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         arm along +ve x-axis configuration
%
% Reference::
% - https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/actual-center-of-mass-for-robot-17264/
%
% Notes::
% - SI units of metres are used.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also mdl_ur3, mdl_ur10, mdl_puma560, SerialLink.

% MODEL: Universal Robotics, UR5, 6DOF, standard_DH

% Copyright (C) 1993-2017, by Peter I. Corke
%
% http://www.petercorke.com

function [r] = mdl_ur5_harvey()
    
    deg = pi/180;
    
    % robot DH parameters
    a     = [0, 0, -0.42500, -0.39225, 0, 0, 0]'; % [m]
    d     = [0, 0.089459, 0, 0, 0.10915, 0.09465, 0.0823]'; % [m]
    alpha = [1.570796327, 1.570796327, 0, 0, 1.570796327, -1.570796327, 0]'; % [rad]
    theta = zeros(7,1); % [rad]
    
    % type joint
    type = [1 0 0 0 0 0 0]'; % 0 revolute, 1 prismatic
    
    % combine robot info
    DH = [theta d a alpha type];

    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.

    robot = SerialLink(DH, ...
        'name', 'UR5_harvey', 'manufacturer', 'Universal Robotics');
    
    % output robot
    r = robot;
    
%     % place the variables into the global workspace
%     if nargin == 1
%         r = robot;
%     elseif nargin == 0
%         assignin('caller', 'ur5_harvey', robot);
%         assignin('caller', 'qz', [0 0 0 0 0 0]); % zero angles
%         assignin('caller', 'qr', [180 0 0 0 90 0]*deg); % vertical pose as per Fig 2
%     end
end
