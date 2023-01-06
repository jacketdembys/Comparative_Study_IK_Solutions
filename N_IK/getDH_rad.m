% Retrieve the DH table of a robotic manipulator.
% Example: DH = getDH(robot, Q_initial)
% Inputs:  robot = a string representing the robot to load
%          Q_initial = a vector representing the initial joint
%          configuration of the robot to load
% Outputs: DH = a matrix representing the corresponding DH table

function DH = getDH_rad(robot, Q_initial, unit_chosen, num_modules)

    
    if (strcmp(robot, 'RRRRRRR'))
        
        DH = [Q_initial(1),             0.0,        0.0,        deg2rad(-90);
              Q_initial(2),             0.0,        0.0,        deg2rad(90);
              Q_initial(3),             0.55,       0.045,      deg2rad(-90);
              Q_initial(4),             0.0,       -0.045,      deg2rad(90);
              Q_initial(5),             0.3,        0.0,        deg2rad(-90);
              Q_initial(6),             0.0,        0.0,        deg2rad(90);
              Q_initial(7),             0.06,       0.0,        deg2rad(0)];
        
        % convert the entries of the DH table
        DH(:, 2) = DH(:, 2) * unit_chosen;
        DH(:, 3) = DH(:, 3) * unit_chosen; 
        
    elseif (strcmp(robot, 'SB'))
        
        DH = [0,     0.0,       0.082,        deg2rad(0);
              0,   0.082,         0.0,        deg2rad(90);
              0,     0.0,       0.084,        deg2rad(0)];
        DH = repmat(DH, num_modules, 1);   
        DH(:,1) = Q_initial;
       
    end     

end