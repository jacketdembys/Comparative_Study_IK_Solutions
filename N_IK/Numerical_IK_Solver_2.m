%% prepare workspace
clc, clear, close all

%% load the tools
addpath petercorke_rtb10/rtb
addpath petercorke_rtb10/smtb
addpath petercorke_rtb10/common


%% initialize global parameters and running modes
rng(1)
robot = 'SB';
%experiments = linspace(3,180,(180-3+1));
experiments = linspace(118, 180,(180-118+1));


mode_run = "run";                           % "run" or "debug"
mode_save = "True";
mode_save_path = "False";

%% choose the number of samples and iterations for "debug" or "run" modes
if strcmp(mode_run, "debug")
    total = 1;
else
    total = length(experiments);
end

for m = 1:total
    
    num_modules = experiments(m);

    dimension = 6;
    str_unit_chosen = "m";
    if(str_unit_chosen == "m")                     % choose the m
        unit_chosen = 1;                           
    elseif (str_unit_chosen == "dm")               % choose the dm
        unit_chosen = 10;
    elseif (str_unit_chosen == "cm")               % choose the cm
        unit_chosen = 100;
    elseif (str_unit_chosen == "mm")               % choose the mm
        unit_chosen = 1000;
    end

    numPoints = 1000;
    data_points = zeros(numPoints, 3*num_modules);
    Q_initial_algo = zeros(numPoints, 3*num_modules);
    
    fprintf("\nBeginning the data generation: %d SB modules", num_modules);
    for i=1:numPoints   
        %% get joint values
        a = -deg2rad(0); b = deg2rad(180);    
        Q_initial = (b-a).*rand(3*num_modules,1) + a;

        %% compute pose 
        %Q_initial = [t1; t2; t3; t4; t5; t6; t7];
        DH = getDH_rad(robot, Q_initial, unit_chosen, num_modules);


        pc_robot_configuration = getRobotConfiguration(robot, unit_chosen, DH);
        T = fkine(pc_robot_configuration, Q_initial);
        D_current = getPose_rad(T, dimension);      

        %T2 = forwardKinematics_rad(DH);
        %D_current = getPose_rad(T, dimension);

        %data_points(i, :) = [Q_initial', D_current'];
        data_points(i, :) = Q_initial;

        %% Initial position        
        a = -deg2rad(0); b = deg2rad(180);    
        t = (b-a).*rand(3*num_modules,1) + a;

        Q_initial_algo(i, :) = t;

    end 


    data_points_all = data_points;
    data_points_init = Q_initial_algo;



    %% rest of the algorithms
    units  = ["m"];  
    inverses = ["MP"]; 
    jacobian_type = 'geometric';                % "numerical", "geometric", "analytical"
    motion = strcat('Results_',num2str(num_modules),'_',robot,'_',jacobian_type,'_jacob');

    total_computation = 0;                      % total computation time

    %% create a folder to store the results
    % motion is the name of the directory when the result are stored
    if ~exist(motion, 'dir')
        mkdir(motion)
    end


    %% load the related sample points and initialize the summary matrices
    samples = length(data_points_init);
    summaryTable = zeros(samples,8);
    %{
    if (strcmp(robot, 'SB'))    
        load(strcat('data_points_init_',num2str(num_modules),'_SB.mat'));
        load(strcat('data_points_all_',num2str(num_modules),'_SB.mat'));
        samples = length(data_points_init);
        summaryTable = zeros(samples,8);
    end
    %}
    
    if strcmp(mode_run, "debug")
        samples = 1;
        iterations = 100;
    else
        iterations = 1000;
    end

    %% loop through the inverses
    for j1=1:length(inverses)

        %% loop through the units
        for i1=1:length(units)

            %% loop through the choice of alpha
            indexes = [1];   %, 0.1, 0.01];

            for j = 1:length(indexes)

                % set the unit, inverse, and alpha choices
                str_unit_chosen = units(i1);
                inverse_chosen = inverses(j1);
                alpha = indexes(j);

                %% choose the unit to investigate
                if(str_unit_chosen == "m")                     % choose the m
                    unit_chosen = 1; 
                    unit_applied = 1000;
                elseif (str_unit_chosen == "dm")               % choose the dm
                    unit_chosen = 10;
                    unit_applied = 100;
                elseif (str_unit_chosen == "cm")               % choose the cm
                    unit_chosen = 100;
                    unit_applied = 10;
                elseif (str_unit_chosen == "mm")               % choose the mm
                    unit_chosen = 1000;
                    unit_applied = 1;
                end   

                if strcmp(mode_save_path, "True")
                    diary(char(strcat(motion, '/', inverse_chosen, '_', str_unit_chosen,'_', num2str(alpha), '.txt')));
                end

                %% set IK path information
                for s=1:samples

                    if strcmp(robot, 'SB')                     
                        Q_initial = data_points_init(s,:)';                        
                        Q_final_d = data_points_all(s,:)'; 
                        dim = 6;      
                    end

                    %% (combined) IK pose accepted tolerance
                    maximum_iteration       = iterations;
                    current_iteration       = 0;
                    D_estimated             = zeros(dim, 1);
                    position_error          = 0.001 * unit_chosen;          
                    orientation_error       = 0.02;                          
                    emax                    = 0.02;
                    %alpha = 0.5;

                    %% DH and initial pose
                    DH = getDH_rad(robot, Q_initial, unit_chosen, num_modules);
                    pc_robot_configuration = getRobotConfiguration(robot, unit_chosen, DH);
                    T = fkine(pc_robot_configuration, Q_initial);
                    D_initial = getPose_rad(T, dim);
                    Q_current = Q_initial;
                    D_current = D_initial;
                    D_current_p = D_initial;

                    DH = getDH_rad(robot, Q_final_d, unit_chosen, num_modules);
                    pc_robot_configuration = getRobotConfiguration(robot, unit_chosen, DH);
                    T_final = fkine(pc_robot_configuration, Q_final_d);  
                    D_final = getPose_rad(T_final, dim);

                    if strcmp(mode_save_path, "True")
                        saveToFile(D_current, motion, str_unit_chosen, inverse_chosen, alpha, "w");
                    end


                    %% get the errors for the inverses
                    if strcmp(robot, "RRP")
                        e = D_final - D_current;

                    else

                        if strcmp(jacobian_type, "geometric")
                            ex = D_final(1:3) - D_current(1:3);
                            %{
                            eo = 0.5*(cross(T(1:3,1), T_final(1:3,1)) + ...
                                      cross(T(1:3,2), T_final(1:3,2)) + ...
                                      cross(T(1:3,3), T_final(1:3,3)));
                            %}
                            eo = 0.5*(cross(T.n, T_final.n) + ...
                                      cross(T.o, T_final.o) + ...
                                      cross(T.a, T_final.a));
                            e = [ex; eo];  
                        elseif strcmp(jacobian_type, "numerical")    
                            e = D_final - D_current;
                        end
                    end

                    %% variables for distance calculation
                    ideal_distance = 0;
                    performed_distance = 0;
                    final = 0;

                    fprintf("\nBeginning the iterative process: sample [%d]", s);
                    tic

                    %% loop to look for the solution
                    while(final == 0)

                        %% compute the traveled distances
                        performed_distance = getDistance(D_current, D_current_p, robot, performed_distance);                
                        D_current_p = D_current;


                        %% DH and current pose
                        %DH = getDH_rad(robot, Q_current, unit_chosen);
                        %pc_robot_configuration = getRobotConfiguration(robot, unit_chosen, DH);
                        %T = fkine(pc_robot_configuration, Q_current);
                        %D_current = getPose_rad(T, length(D_final));


                        %% get the errors for the inverses
                        %{
                        ex = D_final(1:3) - D_current(1:3);
                        eo = 0.5*(cross(T(1:3,1), T_final(1:3,1)) + ...
                                  cross(T(1:3,2), T_final(1:3,2)) + ...
                                  cross(T(1:3,3), T_final(1:3,3)));
                        e = [ex; eo];
                        %}

                        % display the current estimated joint and pose vector
                        if strcmp(mode_run, "debug")
                            displayJoints(Q_current, robot);
                            displayPose(D_current, length(D_final));
                        end

                        %% get the Jacobian                        
                        J = getJacobianMatrix(DH, D_final, D_current, Q_current, robot, jacobian_type, unit_chosen, num_modules);

                        % Display the current jacobian
                        if strcmp(mode_run, "debug")
                            fprintf("\nCurrent Jacobian:")
                            J  %#ok<NOPTS>
                            %J = round(J, 4)
                        end

                        %% check if a singularity happened
                        ss = checkIfSingularityHappened(J, Q_current, current_iteration);

                        if strcmp(inverse_chosen, "SD")
                            %% get d_D and d_Q
                            gamma_max = 0.5;
                            d_Q = select_dampinv2(J, gamma_max, alpha*e) %#ok<NOPTS>
                            d_Q = round(d_Q, 4) %#ok<NOPTS>
                        else

                            %% get the inverse Jacobian
                            inv_J = getInverseJacobian(J, inverse_chosen, e, robot);
                            d_D = alpha*e;

                            %% get d_D and d_Q
                            d_Q = inv_J*d_D;

                            if strcmp(mode_run, "debug")
                                fprintf("\nInverse Jacobian:")
                                inv_J  %#ok<NOPTS>

                                fprintf("\nd_D - current difference in pose:")
                                d_D %#ok<NOPTS>

                                fprintf("\nd_Q - current difference in joint:")
                                d_Q %#ok<NOPTS>
                                %d_Q = wrapToPi(d_Q)%#ok<NOPTS>
                                %d_Q = round(d_Q, 4)%#ok<NOPTS>
                            end

                        end

                        %% get the current (new) joint configuration Q_current
                        Q_current = Q_current + d_Q; 
                        %Q_current = wrapToPi(Q_current);

                        %% estimate the current combined pose error
                        DH = getDH_rad(robot, Q_current, unit_chosen, num_modules);
                        pc_robot_configuration = getRobotConfiguration(robot, unit_chosen, DH);
                        T = fkine(pc_robot_configuration, Q_current);
                        D_current = getPose_rad(T, dim);               


                        %% get the errors for the inverses
                        if strcmp(robot, "RRP")
                            e = D_final - D_current;

                        else

                            if strcmp(jacobian_type, "geometric")
                                ex = D_final(1:3) - D_current(1:3);
                                %{
                                eo = 0.5*(cross(T(1:3,1), T_final(1:3,1)) + ...
                                          cross(T(1:3,2), T_final(1:3,2)) + ...
                                          cross(T(1:3,3), T_final(1:3,3)));
                                %}

                                eo = 0.5*(cross(T.n, T_final.n) + ...
                                      cross(T.o, T_final.o) + ...
                                      cross(T.a, T_final.a));
                                e = [ex; eo];   
                            elseif strcmp(jacobian_type, "numerical")    
                                e = D_final - D_current; 
                            end

                        end

                        %norm(e)                                                       
                        %% track the number of iterations
                        current_iteration = current_iteration + 1;
                        if strcmp(mode_run, "debug")
                            fprintf("\n\nCurrent iteration [%d]\n", current_iteration);  
                        end

                        %% stopping criteria to break out of the while: maximum_iteration or desired error reached
                        %{
                        if or(norm(e)< emax, current_iteration > maximum_iteration)
                            final = 1;
                        end
                        %}

                        if strcmp(robot, 'RRP')
                            if (abs(D_final(1) - D_current(1)) < position_error) && (abs(D_final(2) - D_current(2)) < position_error)  || current_iteration > maximum_iteration 
                                final = 1;
                            end
                        end

                        if strcmp(robot, 'RRPRRRR') || strcmp(robot, 'RRRRRRR') || strcmp(robot, 'SB')
                            if (abs(D_final(1) - D_current(1)) < position_error) && (abs(D_final(2) - D_current(2)) < position_error) && (abs(D_final(3) - D_current(3)) < position_error) && (abs(D_final(4) - D_current(4)) < orientation_error) && (abs(D_final(5) - D_current(5)) < orientation_error) && (abs(D_final(6) - D_current(6)) < orientation_error) || current_iteration > maximum_iteration 
                                final = 1;
                            end
                        end


                        %% save path
                        if strcmp(mode_save_path, "True")
                            saveToFile(D_current, motion, str_unit_chosen, inverse_chosen, alpha, "a");
                        end





                    end % end while loop for maximum iterations

                    %% check if the solution was found before recording the results
                    if (current_iteration < maximum_iteration)
                        solutionFound = 1;
                    else
                        solutionFound = 0;
                    end 

                    %% summary of the IK search solution
                    fprintf("######################################################\n\n")
                    fprintf("Estimated Joint Vector:")
                    Q_current %#ok<NOPTS

                    fprintf("Initial Joint Vector:")
                    Q_initial %#ok<NOPTS

                    fprintf("Initial Pose Vector:")
                    D_initial %#ok<NOPTS

                    fprintf("Desired Pose Vector:")
                    D_final %#ok<NOPTS>

                    fprintf("Estimated Pose Vector:")
                    D_current %#ok<NOPTS>

                    if strcmp(robot, "RRP")
                        Position_error = sqrt(sum((D_final - D_current).^2));
                        fprintf("Position error: %f mm", (Position_error*unit_applied));
                    else
                        Position_error = sqrt(sum((D_final(1:3) - D_current(1:3)).^2));
                        fprintf("Position error: %f mm", (Position_error*unit_applied));

                        Orientation_error = sum(abs(D_final(4:6) - D_current(4:6)))/3;
                        fprintf("\nOrientation error: %f degrees", rad2deg(Orientation_error));
                    end

                    fprintf("\n\nTotal number of iterations: %d", current_iteration);   

                    fprintf("\n")
                    toc %finishes recording time
                    elapsedTime = toc;

                    %% save the summary results in a table
                    if strcmp(robot, 'RRRRRRR')
                        ideal_distance = sqrt(sum((D_final(1:3) - D_initial(1:3)).^2));
                        summaryTable(s, :) = [Q_initial; ...
                                                D_initial; ...
                                                D_final; ...
                                                D_current; ...
                                                Position_error; Orientation_error; ...
                                                current_iteration; elapsedTime; solutionFound; ...
                                                ideal_distance; performed_distance; ss];  
                    end

                    if strcmp(robot, 'SB')
                        ideal_distance = sqrt(sum((D_final(1:3) - D_initial(1:3)).^2));
                        %{
                        summaryTable(s, :) = [Q_initial; ...
                                                D_initial; ...
                                                D_final; ...
                                                D_current; ...
                                                Position_error; Orientation_error; ...
                                                current_iteration; elapsedTime; solutionFound; ...
                                                ideal_distance; performed_distance; ss];
                                            %}
                        summaryTable(s, :) = [Position_error; Orientation_error; ...
                                                current_iteration; elapsedTime; solutionFound; ...
                                                ideal_distance; performed_distance; ss]; 
                    end


                    total_computation = total_computation + elapsedTime;

                end % end of the samples loop




            end % end of the alpha loop   

            %% save the tables of the results to be analyzed
            %{
            if strcmp(mode_save, "True")
                if strcmp(robot, 'RRRRRRR')
                    varNames = {'Q1', 'Q2', 'Q3','Q4', 'Q5', 'Q6','Q7', ...
                                'D1_initial', 'D2_initial','D3_initial', 'D4_initial','D5_initial', 'D6_initial', ...
                                'D1_final', 'D2_final','D3_final', 'D4_final','D5_final', 'D6_final', ...
                                'D1_estimated', 'D2_estimated','D3_estimated', 'D4_estimated','D5_estimated', 'D6_estimated', ...
                                'Final_position_error', 'Final_orientation_error','Total_iterations', ...
                                'Computation_time', 'Solution_found', ...
                                'ideal_distance', 'performed_distance', 'singularity'};

                    dataset_t = table(summaryTable(:,1), summaryTable(:,2), summaryTable(:,3), summaryTable(:,4), summaryTable(:,5), summaryTable(:,6), summaryTable(:,7), ...
                                      summaryTable(:,8), summaryTable(:,9), summaryTable(:,10), summaryTable(:,11), summaryTable(:,12), summaryTable(:,13),...
                                      summaryTable(:,14), summaryTable(:,15), summaryTable(:,16), summaryTable(:,17), summaryTable(:,18), summaryTable(:,19), ...
                                      summaryTable(:,20), summaryTable(:,21), summaryTable(:,22), summaryTable(:,23), summaryTable(:,24), summaryTable(:,25),...
                                      summaryTable(:,26), summaryTable(:,27),...
                                      summaryTable(:,28), summaryTable(:,29),...
                                      summaryTable(:,30), summaryTable(:,31),summaryTable(:,32), summaryTable(:,33),...
                                      'VariableNames',varNames);
                    writetable(dataset_t, strcat(motion,'/','results_',robot,'_',str_unit_chosen,'_', inverse_chosen ,'_', num2str(alpha) ,'.csv'),'Delimiter',',')
                end

                if strcmp(robot, 'SB')
                    varNames = {'Q1', 'Q2', 'Q3','Q4', 'Q5', 'Q6','Q7', 'Q8','Q9', ...
                                'D1_initial', 'D2_initial','D3_initial', 'D4_initial','D5_initial', 'D6_initial', ...
                                'D1_final', 'D2_final','D3_final', 'D4_final','D5_final', 'D6_final', ...
                                'D1_estimated', 'D2_estimated','D3_estimated', 'D4_estimated','D5_estimated', 'D6_estimated', ...
                                'Final_position_error', 'Final_orientation_error','Total_iterations', ...
                                'Computation_time', 'Solution_found', ...
                                'ideal_distance', 'performed_distance', 'singularity'};

                    dataset_t = table(summaryTable(:,1), summaryTable(:,2), summaryTable(:,3), summaryTable(:,4), summaryTable(:,5), summaryTable(:,6), summaryTable(:,7), ...
                                      summaryTable(:,8), summaryTable(:,9), summaryTable(:,10), summaryTable(:,11), summaryTable(:,12), summaryTable(:,13),...
                                      summaryTable(:,14), summaryTable(:,15), summaryTable(:,16), summaryTable(:,17), summaryTable(:,18), summaryTable(:,19), ...
                                      summaryTable(:,20), summaryTable(:,21), summaryTable(:,22), summaryTable(:,23), summaryTable(:,24), summaryTable(:,25),...
                                      summaryTable(:,26), summaryTable(:,27),...
                                      summaryTable(:,28), summaryTable(:,29),...
                                      summaryTable(:,30), summaryTable(:,31),summaryTable(:,32),summaryTable(:,33),summaryTable(:,34),summaryTable(:,35),  ...
                                      'VariableNames',varNames);
                    writetable(dataset_t, strcat(motion,'/','results_',robot,'_',str_unit_chosen,'_', inverse_chosen ,'_', num2str(alpha) ,'.csv'),'Delimiter',',')
                end
            end
            %}

            writematrix(summaryTable, strcat(motion,'/','results_',robot,'_',str_unit_chosen,'_', inverse_chosen ,'_', num2str(alpha) ,'.csv'),'Delimiter',',')


        end   % end of the units loop
    end % end of the inverse loop

    fprintf("\n\nTotal computation time (minutes):")
    total_computation = total_computation/60 %#ok<NOPTS>
    
end