%% Create Multiple Points for the Robots
%addpath("IKINE/")
save_data = "No";
%% 6DoF
rng(1)
robot = 'RRRRRR';
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

numPoints = 10000;
dataPoints = zeros(numPoints, 12);

tic
for i=1:numPoints    
      
    %% get joint values
    %a = -deg2rad(360); b = deg2rad(360);   % unconstrained
    a = -deg2rad(90); b = deg2rad(90);    % constrained
    t1 = (b-a).*rand(1,1) + a;
    
    a = deg2rad(50); b = deg2rad(310);
    %a = deg2rad(50); b = deg2rad(155);
    t2 = (b-a).*rand(1,1) + a;
    
    a = deg2rad(19); b = deg2rad(341);
    %a = deg2rad(19); b = deg2rad(170);
    t3 = (b-a).*rand(1,1) + a;
    
    a = -deg2rad(360); b = deg2rad(360);
    %a = -deg2rad(90); b = deg2rad(90);    % constrained
    t4 = (b-a).*rand(1,1) + a;
    
    a = -deg2rad(360); b = deg2rad(360);
    %a = -deg2rad(90); b = deg2rad(90);    % constrained
    t5 = (b-a).*rand(1,1) + a;
    
    a = -deg2rad(360); b = deg2rad(360);
    %a = -deg2rad(90); b = deg2rad(90);    % constrained
    t6 = (b-a).*rand(1,1) + a;
    
    %% compute pose 
    Q_initial = [t1; t2; t3; t4; t5; t6];
    
    %{
    DH = getDH(robot, Q_initial, unit_chosen);
    T = forwardKinematics(DH);
    D_current = getPose(T, dimension);
    dataPoints(i, :) = [Q_initial', D_current'];
    %}
       
    
    
    DH = getDH_rad_cec(robot, Q_initial,unit_chosen);
    T = forwardKinematics_rad_cec(DH);
    %pc_robot_configuration = getRobotConfiguration_cec(robot, DH);
    %T = fkine(pc_robot_configuration, Q_initial);  
    D_current = getPose_rad_cec(T, dimension);  
    dataPoints(i, :) = [D_current', Q_initial']; 
    
    
    

end 
toc 

%% Visualize the workspace
figure(1)
scatter3(dataPoints(:,1), dataPoints(:,2), dataPoints(:,3), '.')
title("Workspace Kinova 6DoF")
xlabel("X (mm)")
ylabel("Y (mm)")
zlabel("Z (mm)")

%% save data as a csv file
if strcmp(save_data, "Yes")
    varNames = {'x', 'y', 'z','ro', 'pi', 'ya','Q1','Q2','Q3','Q4','Q5','Q6'};   
    dataset_t = table(dataPoints(:, 1), dataPoints(:, 2), dataPoints(:, 3), dataPoints(:, 4),dataPoints(:, 5),dataPoints(:, 6), ...
                      dataPoints(:, 7), dataPoints(:, 8), dataPoints(:, 9), dataPoints(:, 10),dataPoints(:, 11),dataPoints(:, 12), ...
                      'VariableNames',varNames);
    writetable(dataset_t, strcat('cec_data_points_', robot ,'.csv'),'Delimiter',',')
end
           
