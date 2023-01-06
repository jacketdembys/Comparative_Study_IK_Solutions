%% Prepare workspare
clc, clear, close all

%% Experiments
%experiments = linspace(3,180,60);
experiments = linspace(3,180,(180-3+1));

%% Plots the average position errors
str_unit_chosen = "m";
inverse = "MP";
robot = "SB";
jacobian = "geometric";
alpha = 1;


if(str_unit_chosen == "m")                     % choose the m
    unit_chosen = 1000;                           
elseif (str_unit_chosen == "dm")               % choose the dm
    unit_chosen = 100;
elseif (str_unit_chosen == "cm")               % choose the cm
    unit_chosen = 10;
elseif (str_unit_chosen == "mm")               % choose the mm
    unit_chosen = 1;
end 


total = length(experiments);
%total = 1;
results_temp = zeros(total, 6);

for i = 1:total
    
    %% load the results per experiments
    num_modules = experiments(i);
    results = readtable(strcat('Results_', num2str(num_modules),'_', robot,'_',jacobian,'_jacob/results_', robot,'_', str_unit_chosen,'_', inverse,'_', num2str(alpha),'.csv'));
    results = table2array(results);
    
    %% compute metrics
    % Find pourcentage of solution found (unitsratio(to,from))
    countSolutionFound = sum(results(:,5) == 1);  %30
    pourcentageSolutionFound = 100*(countSolutionFound/length(results(:,5)));

    % Find average computation time (s)
    indexes = find(results(:,5)==1);
    %averageComputationTime = 1000*mean(results(indexes,4));
    averageComputationTime = mean(results(indexes,4));

    % Find average position error (mm) and orientation error (degree)
    averagePositionError = mean(results(indexes,1))*unit_chosen;
    %averageOrientationError = rad2deg(mean(results(indexes,2)));
    averageOrientationError = (mean(results(indexes,2)));
    averageOrientationError_dist = (averageOrientationError*1000)/2;
    positioningError = mean([averagePositionError, averageOrientationError_dist]);
    % Find average number of iterations
    averageIterations = round(mean(results(indexes,3)));

    % Find average ideal distance
    averageIdealDistance = mean(results(indexes,6))*unit_chosen;

    % Find average performrf distance
    averagePerformedDistance = mean(results(indexes,7))*unit_chosen;
    
    %% store the results to be plotted
    results_temp(i, :) = [num_modules, positioningError, averagePositionError, averageOrientationError, averageComputationTime, averageIterations]; 
    
end

%% Plots the average pose errors
DoF_module = 3;
x = results_temp(:,1)*DoF_module;
y = results_temp(:,2);

f1 = figure(1);
f1.Position = [50 50 650 500];
plot(x,y,"-","LineWidth",2,...
    "MarkerEdgeColor","blue","MarkerFaceColor",[0.65 0.85 0.90])
xlabel("Number of DoF")
ylabel("Average Pose Errors (mm)")
ax = gca; 
ax.FontSize = 20;
grid on
lgd = legend("NIK");
lgd.Location = 'northeast';



%% Plots the average iterations
x = results_temp(:,1)*DoF_module;
y = results_temp(:,6);

f2 = figure(2);
f2.Position = [620 50 650 500];
plot(x,y,"-","LineWidth",2,...
    "MarkerEdgeColor","blue","MarkerFaceColor",[0.65 0.85 0.90])
xlabel("Number of DoF")
ylabel("Average Iterations")
ylim([0 12])
ax = gca; 
ax.FontSize = 20;
grid on
lgd = legend("NIK");
lgd.Location = 'northeast';



%% Plots the average runtime errors
x = results_temp(:,1)*DoF_module;
y = results_temp(:,5);

f3 = figure(3);
f3.Position = [1200 50 650 500];
plot(x,y,"-","LineWidth",2,...
    "MarkerEdgeColor","blue","MarkerFaceColor",[0.65 0.85 0.90])
xlabel("Number of DoF")
ylabel("Average Runtime (s)")
ax = gca; 
ax.FontSize = 20;
grid on
lgd = legend("NIK");
lgd.Location = 'northwest';

