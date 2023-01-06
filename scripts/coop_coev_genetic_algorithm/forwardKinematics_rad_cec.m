%% ViGIR - Laboratory - May 2020
% Complete Forward-Kinematics of Robotic Manipulators
% The DH is the Denhavit-Hartenberg (D-H) Table of the manipullator 
% The manipulator DoF is expressed by the number of rows
function T = forwardKinematics_rad_cec(DH)

    % get the DoF of the robot
    [DoF, ~] = size(DH);
    
    % extract the DH parameters
    Q = DH(:, 1);
    d = DH(:, 2);
    a = DH(:, 3);
    al = DH(:, 4);
    
    % initialize empty matrices
    A = zeros(4, 4, DoF);
    
    % compute the total transformation matrix
    for i=1:DoF
        
        % find the transformation matrix A from frame i to frame i+1
        A(:,:,i) = [cos(Q(i)),      -sin(Q(i))*cos(al(i)),       sin(Q(i))*sin(al(i)),          a(i)*cos(Q(i));
                    sin(Q(i)),       cos(Q(i))*cos(al(i)),      -cos(Q(i))*sin(al(i)),          a(i)*sin(Q(i));
                            0,                 sin(al(i)),                 cos(al(i)),                    d(i);
                            0,                          0,                          0,                       1];
    end
    
    T = eye(4, 4);
    for i=1:DoF
        T = T * A(:, :, i);
    end
    
    %% avoid numberical precision errors
    %T = round(T, 6);
    
end