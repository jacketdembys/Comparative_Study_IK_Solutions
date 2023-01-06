%% Create Multiple Points for the SuperBot Robots
addpath petercorke_rtb10/rtb
addpath petercorke_rtb10/smtb
addpath petercorke_rtb10/common


rng(1)
num_modules = 3;
robot = 'SB';

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
data_points = zeros(numPoints, 3*num_modules + 6);
Q_initial_algo = zeros(numPoints, 3*num_modules);
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
       
    data_points(i, :) = [Q_initial', D_current'];
        
    %% Initial position        
    a = -deg2rad(0); b = deg2rad(180);    
    t = (b-a).*rand(3*num_modules,1) + a;
    
    Q_initial_algo(i, :) = t;

end 


data_points_all = data_points;
data_points_init = Q_initial_algo;

scatter3(data_points(:,10), data_points(:,11), data_points(:,12), '.')


filename = strcat('data_points_all_', num2str(num_modules), '_', robot,'.mat');
filename_algo = strcat('data_points_init_', num2str(num_modules), '_', robot,'.mat');
%% save the variable
save(filename, 'data_points_all')
save(filename_algo, 'data_points_init')
    
