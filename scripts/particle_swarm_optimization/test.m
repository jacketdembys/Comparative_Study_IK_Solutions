 %% Prepare the workspace
clc, close all, clear all

%% Problem definition
problem.CostFunction = @(x) Sphere(x);          % Cost function
problem.nVar = 5;                               % Number of unknowns or decision variables
problem.VarMin = 3;                             % Lower Bound of decision variables
problem.VarMax = 10;                            % Upper Bound of decision variables


%% Parameters of PSO
% constriction coefficients from Clerk and Kennedy, 2002
use_constrictions = false;
kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2;
chi = (2*kappa)/abs(2-phi-sqrt(phi^2-4*phi));

% now setting the parameters
params.MaxIt = 1000;                            % Maximum number of iterations
params.nPop = 50;                               % Population size or Swarm Size
params.ShowIterInfo = true;                     % Flag for showing iteration info

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
out = minimize_with_particle_swarm(problem, params);
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