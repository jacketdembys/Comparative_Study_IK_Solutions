% Retrieve the DH table of a robotic manipulator.
% Example: DH = getDH(robot, Q_initial)
% Inputs:  robot = a string representing the robot to load
%          Q_initial = a vector representing the initial joint
%          configuration of the robot to load
% Outputs: DH = a matrix representing the corresponding DH table

function DH = getDH_rad_cec(robot, Q_initial, U)

    
    if (strcmp(robot, 'RRRRRR'))   
        
        D1 = 0.2755;
        D2 = 0.4100;
        D3 = 0.2073;
        D4 = 0.0741;
        D5 = 0.0741;
        D6 = 0.1600;
        e2 = 0.0098;
        aa = ((30.0*pi)/180.0);
        ca = cos(aa);
        sa = sin(aa);
        c2a = cos(2*aa);
        s2a = sin(2*aa);
        d4b = (D3 + sa/s2a *D4);
        d5b = (sa/s2a*D4 + sa/s2a *D5);
        d6b = (sa/s2a*D5 + D6);
        
        DH = [-Q_initial(1),             D1,          0.0,      pi/2;
               Q_initial(2)+(pi/2),      0.0,         D2,       pi;
               Q_initial(3)-(pi/2),     -e2,          0.0,      pi/2;
               Q_initial(4),            -d4b,         0.0,      2*aa;
               Q_initial(5)+(pi),       -d5b,         0.0,      2*aa;
               Q_initial(6)-(pi/2),     -d6b,         0.0,      pi];  
           
        DH(:,2) = U*DH(:,2);
        DH(:,3) = U*DH(:,3);
       
    end     

end