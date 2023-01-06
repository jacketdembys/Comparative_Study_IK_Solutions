function obj = IK_cost(sol)

    t1 = sol(1);
    t2 = sol(2);
    t3 = sol(3);
    t4 = sol(4);
    t5 = sol(5);
    [x,y,z] = Forward(t1,t2,t3,t4,t5);
    
    % Cartesian point coordinate [x,x,z]
    v = [495,0,255.55];
    
    % Objective function
    obj = sqrt((x-v(1))^2+(y-v(2))^2+(z-v(3))^2); 

end