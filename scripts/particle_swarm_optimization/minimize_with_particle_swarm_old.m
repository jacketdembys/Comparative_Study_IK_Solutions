%% Prepare the workspace
clc, close all, clear all

%% Problem definition
CostFunction = @(x) Sphere(x);          % Cost function
nVar = 5;                               % Number of unknowns or decision variables
VarSize = [1 nVar];                     % Matrix Size of the decision variables
VarMin = -10;                           % Lower Bound of decision variables
VarMax = 10;                            % Upper Bound of decision variables


%% Parameters of PSO
MaxIt = 1000;                            % Maximum number of iterations
nPop = 50;                              % Population size or Swarm Size
w = 1;                                  % Inertia coefficient
wdamp = 0.99;                           % damping ratio of inertia coefficient
c1 = 2;                                 % Personal acceleration coefficient (personal learning)   
c2 = 2;                                 % Global or social acceleration coefficient (global learning)       


%% Initialization

% create particle template
empty_particle.Position = [];           % Position of the particle
empty_particle.Velocity = [];           % Velocity of the particle
empty_particle.Cost = [];               % Cost function evaluated at the particle
empty_particle.Best.Position = [];      % Personal best position of the particle
empty_particle.Best.Cost = [];          % Personal best cost of the particle

% create population array
particle = repmat(empty_particle, nPop, 1);

% initialize global best
GlobalBest.Cost = inf;

% initialize population member
for i=1:nPop
    
    % generate random solution
    particle(i).Position = unifrnd(VarMin, VarMax, VarSize);
    
    % initialize velocity
    particle(i).Velocity = zeros(VarSize);
    
    % evaluation
    particle(i).Cost = CostFunction(particle(i).Position);
    
    % update the particle best
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
    
    % update the global best (global memory of the whole swarm)
    if particle(i).Best.Cost < GlobalBest.Cost
        GlobalBest = particle(i).Best;
    end
    
end

% Array to hold best costs values on each iterations
BestCosts = zeros(MaxIt, 1);

%% Main loop of PSO
for it = 1:MaxIt
    
    for i = 1:nPop
        
        % update particle velocity
        particle(i).Velocity = w*particle(i).Velocity ...
                               + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position) ...
                               + c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);
        
        % update particle position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % update cost evaluation
        particle(i).Cost = CostFunction(particle(i).Position);
        
        % update personal best
        if particle(i).Cost < particle(i).Best.Cost
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;
            
            % update global best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest = particle(i);
            end
            
        end
        
    end
  
    % store the best cost value 
    BestCosts(it) = GlobalBest.Cost;
    
    % display iteration information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
    
    % damp inertia coefficient;
    w = w * wdamp;
    
end


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