function J =  getJacobianMatrix(DH, D_final, D_current, Q_current, robot, jacobian_type, unit_chosen, num_modules)

     if strcmp(robot, "RRP")

         if strcmp(jacobian_type, "geometric")

            T_all = getForwardT_rad(DH);
            J = getGeometricJ_rad(T_all, length(D_final), robot);
            J = J(1:length(D_final),:);

         elseif strcmp(jacobian_type, "analytical")

            a1 = 1 * unit_chosen;
            a2 = 1.1 * unit_chosen;

            s1 = sin(Q_current(1));
            c1 = cos(Q_current(1));
            d = Q_current(3);
            s12 = sin(Q_current(1) + Q_current(2));
            c12 = cos(Q_current(1) + Q_current(2));
            J = [-a1*s1 - a2*s12 + d*c12, -a2*s12 + d*c12, s12;
                  a1*c1 + a2*c12 + d*s12, a2*c12 + d*s12, -c12];

         end
     else


         if strcmp(jacobian_type, "geometric")

            %{
            T_all = getForwardT_rad(DH);
            J = getGeometricJ_rad(T_all, length(D_final), robot);
            J = J(1:length(D_final),:);
             %}
            pc_robot_configuration = getRobotConfiguration(robot, unit_chosen, DH);
            J = jacob0(pc_robot_configuration, Q_current);


         end
     end
 
 
    if strcmp(jacobian_type, "numerical")

        % set the perturbations values per joints
        delta_Q = zeros(length(Q_current), 1);
        for c=1:length(Q_current)
            delta_Q(c) = deg2rad(1); 
        end

        % build the numerical jackobian
        D_temp = zeros(length(D_final), length(Q_current));        
        J = zeros(length(D_final), length(Q_current));
        for k=1:length(Q_current)
            Q_temp = Q_current;
            Q_temp(k) = Q_temp(k) + delta_Q(k);
            DH = getDH_rad(robot, Q_temp, unit_chosen, num_modules);
            
            pc_robot_configuration = getRobotConfiguration(robot, unit_chosen, DH);
            T = fkine(pc_robot_configuration, Q_temp);
            
            %T = forwardKinematics_rad(DH);        
            D_temp(:, k) = getPose_rad(T, length(D_final)); 
            J(:,k) = (D_temp(:, k) - D_current)/delta_Q(k);
        end 
    end
    
end