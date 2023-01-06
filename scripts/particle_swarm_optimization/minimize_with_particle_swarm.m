function out = minimize_with_particle_swarm(problem, params)

    %% Problem definition
    CostFunction = problem.CostFunction;          % Cost function
    nVar = problem.nVar;                          % Number of unknowns or decision variables
    VarSize = [1 nVar];                           % Matrix Size of the decision variables
    VarMin = problem.VarMin;                      % Lower Bound of decision variables
    VarMax = problem.VarMax;                      % Upper Bound of decision variables


    %% Parameters of PSO
    MaxIt = params.MaxIt;                         % Maximum number of iterations
    nPop = params.nPop;                           % Population size or Swarm Size
    w = params.w;                                 % Inertia coefficient
    wdamp = params.wdamp;                         % damping ratio of inertia coefficient
    c1 = params.c1;                               % Personal acceleration coefficient (personal learning)   
    c2 = params.c2;                               % Global or social acceleration coefficient (global learning)       
    ShowIterInfo = params.ShowIterInfo;           % Flag for showing iteration info
    
    MaxVelocity = 0.2*(VarMax - VarMin);
    MinVelocity = -MaxVelocity;

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

    % initialize population members
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
                            
            % apply lower bound and upper bound to particle velocity
            particle(i).Velocity = max(particle(i).Velocity, MinVelocity);
            particle(i).Velocity = min(particle(i).Velocity, MaxVelocity);
                               
            % update particle position
            particle(i).Position = particle(i).Position + particle(i).Velocity;
            
            % apply lower bound and upper bound to particle position
            particle(i).Position = max(particle(i).Position, VarMin);
            particle(i).Position = min(particle(i).Position, VarMax);
              

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
        if ShowIterInfo
            disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
        end

        % damp inertia coefficient;
        w = w * wdamp;

    end

    out.Population = particle;
    out.BestSolution = GlobalBest;
    out.BestCosts = BestCosts;

end