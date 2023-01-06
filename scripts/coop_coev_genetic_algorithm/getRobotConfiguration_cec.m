function r = getRobotConfiguration_cec(robot_chosen, DH)

    if strcmp(robot_chosen, "RRRRRR")
        
        % links: alpha, a, theta, d
        L1 = link([DH(1, 4)    DH(1, 3)     0   DH(1, 2)    0],'standard');
        L2 = link([DH(2, 4)    DH(2, 3)     0   DH(2, 2)    0],'standard');
        L3 = link([DH(3, 4)    DH(3, 3)     0   DH(3, 2)    0],'standard');
        L4 = link([DH(4, 4)    DH(4, 3)     0   DH(4, 2)    0],'standard');
        L5 = link([DH(5, 4)    DH(5, 3)     0   DH(5, 2)    0],'standard');
        L6 = link([DH(6, 4)    DH(6, 3)     0   DH(6, 2)    0],'standard');
                
        r = robot({L1 L2 L3 L4 L5 L6});         
      
    end

end