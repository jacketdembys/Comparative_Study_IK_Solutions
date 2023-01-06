%% set paths and running modes
clc, clear, close all
addpath("scripts/gradient_descent/")
addpath("scripts/genetic_algorithm/")
addpath("scripts/particle_swarm_optimization/")
fmin_choice = "ml";                         % ml (mixed landscape (gemonn))
rng default                                 % For reproducibility

%% set the initial settings
npop = 100;                             % Population size
learning_rate = 0.1;                    % Learning rate for gradient descent
max_steps = 1000;                       % maximum number of iterations for all the algorithms
n = 60;                                 % number of variables or dimensions
rate_mut=0.8;                           % rate of mutation for genetic algorithms
rate_cross=0.4;                         % rate of cross-over for genetic algorithms
lb = -500;                              % Lower bound of the decision variables 
ub = 500;                               % Upper bound of the decision variables    
epsilon = unifrnd(-0.001,0.001,n,1);    % incremental steps for numerical gradient approximation

%% set the function to minimize
if strcmp (fmin_choice , "ml")
    fmin = @(x) sum(x(1:40).^2) - (1/2)*sum(x(41:60).*sin(sqrt(abs(x(41:60)))));
end
    

%% minimize with gradient descent
fprintf("\nMinimizing the function with Gradient Descent\n")

problem.CostFunction = fmin;
problem.nVar = n; 

params.max_steps = max_steps;
params.learning_rate = learning_rate;
params.epsilon = epsilon;    
params.lb = lb;
params.ub = ub;
params.ShowIterInfo = false;

tic
[out] = minimize_with_gradient_descent(problem, params);
toc
recap_obj_gd = out.BestCosts;
fval_gd = out.BestSolution.Cost;


%% minimize with vanilla genetic algorithm
fprintf("\nMinimizing the function with Vanilla Genetic Algorithm\n")
problem.CostFunction = fmin;                        % Cost Function
problem.CrossFunction = @(x1,x2) g_cross(x1,x2);    % Crossover operator (1 single point crossover)
problem.CrossFunctionType = "sp";
problem.nVar = n;                                   % Number of unknowns or decision variables

params.npop = npop;                                 % Population size or Swarm Size
params.rate_mut = rate_mut;                         % rate of mutation
params.rate_cross = rate_cross;                     % rate of crossover
params.max_steps = max_steps;                       % Maximum number of iterations
params.lb = lb;                                     % Lower Bound of decision variables
params.ub = ub;                                     % Upper Bound of decision variables

tic
[out] = minimize_with_genetic_algorithm(problem, params);
toc 
recap_obj_ga = out.BestCosts;
fval_ga = out.BestSolution.Cost;

%% minimize with SBX genetic algorithm
problem.CostFunction = fmin;                        % Cost Function
problem.CrossFunction = @(x1,x2) sbx_cross(x1,x2);  % Crossover operator (sbx operator)
problem.CrossFunctionType = "sbx";
problem.nVar = n;                                   % Number of unknowns or decision variables

params.npop = npop;                                 % Population size or Swarm Size
params.rate_mut = rate_mut;                         % rate of mutation
params.rate_cross = rate_cross;                     % rate of crossover
params.max_steps = max_steps;                       % Maximum number of iterations
params.lb = lb;                                     % Lower Bound of decision variables
params.ub = ub;                                     % Upper Bound of decision variables
params.epsilon = epsilon;

fprintf("\nMinimizing the function with SBX Genetic Algorithm\n")
tic
[out] = minimize_with_genetic_algorithm(problem, params);
toc 
recap_obj_sbx_ga = out.BestCosts;
fval_sbx_ga = out.BestSolution.Cost;

%% minimize with gSBX genetic algorithm
problem.CostFunction = fmin;                                            % Cost Function
problem.CrossFunction = @(x1,x2,g1,g2) gsbx_cross(x1,x2,g1,g2);         % Crossover operator (gsbx operator)
problem.CrossFunctionType = "gsbx";
problem.nVar = n;                                                       % Number of unknowns or decision variables


params.npop = npop;                                 % Population size or Swarm Size
params.rate_mut = rate_mut;                         % rate of mutation
params.rate_cross = rate_cross;                     % rate of crossover
params.max_steps = max_steps;                       % Maximum number of iterations
params.lb = lb;                                     % Lower Bound of decision variables
params.ub = ub;                                     % Upper Bound of decision variables

fprintf("\nMinimizing the function with gSBX Genetic Algorithm\n")
tic
[out] = minimize_with_genetic_algorithm(problem, params);
toc 
recap_obj_gsbx_ga = out.BestCosts;
fval_gsbx_ga = out.BestSolution.Cost;


%% minimize with Vanilla Particle Swarm Optimization
fprintf("\nMinimizing the function with Particle Swarm Optimizarion\n")

problem.CostFunction = fmin;                        % Cost function
problem.nVar = n;                                   % Number of unknowns or decision variables
problem.VarMin = lb;                                % Lower Bound of decision variables
problem.VarMax = ub;                                % Upper Bound of decision variables

use_constrictions = false;
kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2;
chi = (2*kappa)/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt = max_steps;                           % Maximum number of iterations
params.nPop = npop;                                 % Population size or Swarm Size
params.ShowIterInfo = false;                        % Flag for showing iteration info

if use_constrictions
    params.w = chi;                                 % Inertia coefficient (w = 1)
    params.wdamp = 1;                               % damping ratio of inertia coefficient (wdamp = 0.99 w/o constrictions)
    params.c1 = chi*phi1;                           % Personal acceleration coefficient (personal learning)   
    params.c2 = chi*phi2;                           % Global or social acceleration coefficient (global learning)       
else
    params.w = 1;                                   % Inertia coefficient (w = 1)
    params.wdamp = 0.99;                            % damping ratio of inertia coefficient (wdamp = 0.99 w/o constrictions)
    params.c1 = 2;                                  % Personal acceleration coefficient (personal learning)   
    params.c2 = 2;                                  % Global or social acceleration coefficient (global learning)       
end

%% Minimize with PSO
tic
out = minimize_with_particle_swarm(problem, params);
fval_pso = out.BestSolution.Cost;
recap_obj_pso = out.BestCosts;
toc




%% visualizations
figure(1)
hold on
plot(recap_obj_gd)
plot(recap_obj_ga)
plot(recap_obj_sbx_ga)
plot(recap_obj_gsbx_ga)
plot(recap_obj_pso)
hold off
title("Objective Function")
ylabel("Minimum Objective Value")
xlabel("Steps")
lgd = legend("Gradient Descent", "Vanilla Genetic Algorithm", "SBX Genetic Algorithm", "gSBX Genetic Algorithm", "Vanilla Particle Swarm Optimization");



figure(2)
hold on
plot(recap_obj_gd)
plot(recap_obj_gsbx_ga)
hold off
title("Objective Function")
ylabel("Minimum Objective Value")
xlabel("Steps")
lgd = legend("Gradient Descent", "gSBX Genetic Algorithm");

