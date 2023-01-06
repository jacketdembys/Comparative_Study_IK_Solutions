%% Prepare the workspace
clc, clear, close all;

%% Problem definition
problem.CostFunction = @(x) Sphere(x);          % Cost function
problem.nVar = 5;                               % Number of unknowns or decision variables
problem.VarMin = -10;                           % Lower Bound of decision variables
problem.VarMax = 10;                            % Upper Bound of decision variables
problem.CrossFunctionType = "gsbx";              % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)  
rng default                                     % For reproducibility


%% Parameters of GA
params.MaxIt = 100;                              % Maximum number of iterations
params.nPop = 100;                               % Population size or Swarm Size
params.beta = 1;
params.pC = 1;                                   % Percentage of children 
params.mu = 0.02;
params.sigma = 0.1;
params.gamma = 0;
params.epsilon = unifrnd(-0.001,0.001,problem.nVar,1);
params.ShowIterInfo = true;                     % Flag for showing iteration info

%% Minimize with GA
out = minimize_with_traditional_genetic_algorithm(problem, params);
BestSolution = out.BestSolution;
BestCosts = out.BestCosts;


%% Results
figure(1);
subplot(1,2,1), plot(BestCosts, 'LineWidth', 2);
title('Best Cost with Plot')
xlabel('Steps')
ylabel("Minimum Objective Value")
grid on

subplot(1,2,2), semilogy(BestCosts, 'LineWidth', 2);
title('Best Cost with SemiLogy')
xlabel('Steps')
ylabel("Minimum Objective Value")
grid on