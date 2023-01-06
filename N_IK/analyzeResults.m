clc, clear

format shortG
%% Load and analyze the results for the 3DoF robot
str_unit_chosen = "m";
inverse = "MP";
robot = "SB";
num_modules = 60;
jacobian = "geometric";
alpha = 1;
results = readtable(strcat('Results_', num2str(num_modules),'_', robot,'_',jacobian,'_jacob/results_', robot,'_', str_unit_chosen,'_', inverse,'_', num2str(alpha),'.csv'));
results = table2array(results);


if(str_unit_chosen == "m")                     % choose the m
    unit_chosen = 1000;                           
elseif (str_unit_chosen == "dm")               % choose the dm
    unit_chosen = 100;
elseif (str_unit_chosen == "cm")               % choose the cm
    unit_chosen = 10;
elseif (str_unit_chosen == "mm")               % choose the mm
    unit_chosen = 1;
end 



if strcmp(robot, "RRRRRRR")
    % Find pourcentage of solution found (unitsratio(to,from))
    countSolutionFound = sum(results(:,5) == 1); 
    pourcentageSolutionFound = 100*(countSolutionFound/length(results(:,5)))

    % Find average computation time (s)
    indexes = find(results(:,5)==1);
    averageComputationTime = 1000*mean(results(indexes,4))
    %averageComputationTime3DoF = mean(results3DoF(:,12))

    % Find average position error (mm)
    averagePositionError = mean(results(indexes,1))*unit_chosen
    averageOrientationError = rad2deg(mean(results(indexes,2)));

    % Find average orientation error (no orientation for the 3DoF)
    % Find average number of iterations
    averageIterations = round(mean(results(indexes,3)))

    % Find max position errors
    %maxPositionError3DoF = max(results3DoF(:,10))*unit_chosen

    % Find min position errors
    %minPositionError3DoF = min(results3DoF(:,10))*unit_chosen

    % Find average ideal distance
    averageIdealDistance = mean(results(indexes,6))*unit_chosen

    % Find average performrf distance
    averagePerformedDistance = mean(results(indexes,7))*unit_chosen

    % Find min orientation errors (no orientation for the 3DoF)
    % Find max orientation errors (no orientation for the 3DoF)
end


if strcmp(robot, "SB")
    % Find pourcentage of solution found (unitsratio(to,from))
    countSolutionFound = sum(results(:,5) == 1);  %30
    pourcentageSolutionFound = 100*(countSolutionFound/length(results(:,5)))

    % Find average computation time (s)
    indexes = find(results(:,5)==1);
    %averageComputationTime = 1000*mean(results(indexes,4))
    averageComputationTime = mean(results(indexes,4))
    %averageComputationTime3DoF = mean(results3DoF(:,12))

    % Find average position error (mm)
    averagePositionError = mean(results(indexes,1))*unit_chosen;
    %averageOrientationError = rad2deg(mean(results(indexes,2)));
    averageOrientationError = mean(results(indexes,2));
    averageOrientationError_dist = (averageOrientationError*1000)/2;
    positioningError = mean([averagePositionError, averageOrientationError_dist])

    % Find average orientation error (no orientation for the 3DoF)
    % Find average number of iterations
    averageIterations = round(mean(results(indexes,3)))

    % Find max position errors
    %maxPositionError3DoF = max(results3DoF(:,10))*unit_chosen

    % Find min position errors
    %minPositionError3DoF = min(results3DoF(:,10))*unit_chosen

    % Find average ideal distance
    averageIdealDistance = mean(results(indexes,6))*unit_chosen

    % Find average performrf distance
    averagePerformedDistance = mean(results(indexes,7))*unit_chosen

    % Find min orientation errors (no orientation for the 3DoF)
    % Find max orientation errors (no orientation for the 3DoF)
end
