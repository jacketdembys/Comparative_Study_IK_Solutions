function robot = getRobotConfiguration(robot_chosen, unit_chosen, DH)

    if strcmp(robot_chosen, "RRRRRRR")
        
        L(1) = Link('revolute', 'd', 0.0*unit_chosen, 'a', 0.0*unit_chosen, 'alpha', -pi/2);
        L(2) = Link('revolute', 'd', 0.0*unit_chosen, 'a', 0.0*unit_chosen, 'alpha', pi/2);
        L(3) = Link('revolute', 'd', 0.55*unit_chosen, 'a', 0.045*unit_chosen, 'alpha', -pi/2);
        L(4) = Link('revolute', 'd', 0.0*unit_chosen, 'a', -0.045*unit_chosen, 'alpha', pi/2);
        L(5) = Link('revolute', 'd', 0.3*unit_chosen, 'a', 0.0*unit_chosen, 'alpha', -pi/2);
        L(6) = Link('revolute', 'd', 0.0*unit_chosen, 'a', 0.0*unit_chosen, 'alpha', pi/2);
        L(7) = Link('revolute', 'd', 0.06*unit_chosen, 'a', 0.0*unit_chosen, 'alpha', 0);
        robot = SerialLink(L);
        robot.name = '7DoF-7R';       
        
    elseif strcmp(robot_chosen, "SB")
        
        
        %{
            L(1) = Link('revolute', 'd', DH(1,2), 'a', DH(1,3), 'alpha', DH(1,4));
            L(2) = Link('revolute', 'd', DH(2,2), 'a', DH(2,3), 'alpha', DH(2,4));
            L(3) = Link('revolute', 'd', DH(3,2), 'a', DH(3,3), 'alpha', DH(3,4));
            L(4) = Link('revolute', 'd', DH(4,2), 'a', DH(4,3), 'alpha', DH(4,4));
            L(5) = Link('revolute', 'd', DH(5,2), 'a', DH(5,3), 'alpha', DH(5,4));
            L(6) = Link('revolute', 'd', DH(6,2), 'a', DH(6,3), 'alpha', DH(6,4));
            L(7) = Link('revolute', 'd', DH(7,2), 'a', DH(7,3), 'alpha', DH(7,4));
            L(8) = Link('revolute', 'd', DH(8,2), 'a', DH(8,3), 'alpha', DH(8,4));
            L(9) = Link('revolute', 'd', DH(9,2), 'a', DH(9,3), 'alpha', DH(9,4));
            robot = SerialLink(L);
            robot.name = 'SB';
        %}
        
        for i=1:length(DH(:,1))
            L(i) = Link('revolute', 'd', DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4));
        end
        robot = SerialLink(L);
        robot.name = 'SB';
        

    end

end