function obj = IK_cost_b(Q, D_desired, U, d, ot)

    robot = 'RRRRRR';
    DH = getDH_rad_cec(robot, Q, U);
    T = forwardKinematics_rad_cec(DH); 
    D_current = getPose_rad_cec(T, d)';  
        
    if d == 3

        % Objective function
        if ot == "position"
            obj = sqrt(sum((D_desired(1:3)-D_current(1:3)).^2)); 
        end
        
    elseif d == 6 

        % RPY based Objective function
        if ot == "poserpyr"
            %disp("Using poserpyr objective")
            obj_p = sqrt(sum((D_desired(1:3)-D_current(1:3)).^2));             
            obj_o = sum(abs(D_desired(4:6)-D_current(4:6)))/3;    
            r = rand();
            w1 = r;
            w2 = 1-r;
            obj = w1*obj_p + w2*obj_o;
        elseif ot == "poserpys"  
            obj_p = sqrt(sum((D_desired(1:3)-D_current(1:3)).^2));             
            obj_o = sum(abs(D_desired(4:6)-D_current(4:6)))/3;                         
            w1 = 0.5;
            w2 = 0.5;
            obj = w1*obj_p + w2*obj_o;
        elseif ot == "posehtm"
            T_current = T;
            T_desired = Build_H_from_RPY(D_desired);  
            obj_p = sqrt(sum((D_desired(1:3)-D_current(1:3)).^2));
            obj_o =  sum(diag(eye(3))) - sum(diag(T_desired(1:3,1:3)*inv(T_current(1:3,1:3))));               
            r = rand();
            w1 = r;
            w2 = 1-r;
            obj = w1*obj_p + w2*obj_o;
            
            
            %{
            obj_tmp = T_desired*inv(T_current);
            obj = sqrt(sum(diag(eye(4)))) - sqrt(sum(diag(obj_tmp)));            
            obj_ideal = eye(4);
            obj_ideal(4,4) = obj_ideal(4,4)*U; 
            obj = norm(obj_ideal) - norm(obj_tmp);
            %}
        elseif ot == "posequaternion"
            T_current = T;
            T_desired = Build_H_from_RPY(D_desired);  
            obj_p = sqrt(sum((D_desired(1:3)-D_current(1:3)).^2));
            obj_o_c = rotm2quat(T_current(1:3,1:3)); 
            obj_o_d = rotm2quat(T_desired(1:3,1:3));
            obj_o = dot(obj_o_d, obj_o_c);       
            r = rand();
            w1 = r;
            w2 = 1-r;
            obj = w1*obj_p + w2*obj_o;
        end
    end

end