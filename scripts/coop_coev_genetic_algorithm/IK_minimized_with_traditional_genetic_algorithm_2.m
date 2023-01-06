function [out] = IK_minimized_with_traditional_genetic_algorithm_2(problem, params)

        
        % The higher mutation rate increases the exploration ability of the
        % GA but can result to a slow convergence of the algorithm

        % Problem 
        CostFunction = problem.CostFunction;
        nVar = problem.nVar;
        VarSize = [1, nVar];
        VarMin = problem.VarMin;
        VarMax = problem.VarMax;
        crossover_type = problem.CrossFunctionType;
        selection_type = problem.ParentSelectionType;
        DesiredPose = problem.DesiredPose; 
        DesiredTolerance = problem.DesiredTolerance;
        UnitChosen = problem.UnitChosen;
        UnitApplied = problem.UnitApplied;
        Dimension = problem.Dimension;
        ObjectiveType = problem.ObjectiveType;
        
        % Params
        MaxIt = params.MaxIt;
        nPop = params.nPop;
        rateMutation = params.rateMutation;
        rateCross = params.rateCross;
        mu = params.mu;                            
        sigma = params.sigma;  
        gamma = params.gamma;
        epsilon = params.epsilon;
        ShowIterInfo = params.ShowIterInfo;
        
        
        % Template for Empty Individuals
        empty_individual.Position = [];
        empty_individual.Cost = [];
        popAll = repmat(empty_individual, nPop, 1);
        
        % set the number of mutation and cross-over based on the chosen
        % rates
        numMutation = round(nPop*rateMutation);        
        numCross = round(nPop*rateCross);
       
        
        % Best solution ever found
        BestSolution.Cost = inf;
        
        % initialization
        
        for i=1:nPop
            
            % Generate random solution
            popAll(i).Position = unifrnd(VarMin, VarMax, VarSize);
            
            % Evaluate solution
            popAll(i).Cost = CostFunction(popAll(i).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
            
           
            % Compare solution to the best solution ever found
            if popAll(i).Cost < BestSolution.Cost
                BestSolution = popAll(i);
            end
                        
        end
        
        % Best cost of Iterations
        BestCosts = zeros(MaxIt, 1);
        
        % Main loop
        for it=1:MaxIt    
            
            
            % Template to hold the results from mutation and crossover
            popAllM = repmat(empty_individual, numMutation, 1);
            popAllC = repmat(empty_individual, numCross, 2);

            % Mutation
            for m = 1:numMutation
                
                % Perform Mutation
                r = randi(nPop);
                popAllM(m).Position = Mutate(popAll(r).Position, mu, sigma);
                
                % Checks for the bounds
                popAllM(m).Position = max(popAllM(m).Position, VarMin);
                popAllM(m).Position = min(popAllM(m).Position, VarMax);
                
                % Evaluation
                popAllM(m).Cost = CostFunction(popAllM(m).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                
                % Compare solution to the best solution ever found
                if popAllM(m).Cost < BestSolution.Cost
                    BestSolution = popAllM(m);
                end
            end
            
            % Crossover
            for k = 1:numCross
                
                % Select parents
                if selection_type == "random"
                    q = randperm(nPop);
                    p1 = popAll(q(1));
                    p2 = popAll(q(2));
                end
                
                % Perform crossover
                if crossover_type == "singlepoint"
                    [popAllC(k,1).Position, popAllC(k,2).Position] = ...
                        SinglePointCrossover(p1.Position, p2.Position);   
                elseif crossover_type == "doublepoint"
                    [popAllC(k,1).Position, popAllC(k,2).Position] = ...
                        DoublePointCrossover(p1.Position, p2.Position); 
                elseif crossover_type == "uniform"
                    [popAllC(k,1).Position, popAllC(k,2).Position] = ...
                        UniformCrossover(p1.Position, p2.Position, gamma);   
                elseif crossover_type == "combined_sdu"
                    [popAllC(k,1).Position, popAllC(k,2).Position] = ...
                        CombinedCrossover(p1.Position, p2.Position, gamma);    
                elseif crossover_type == "sbx"
                    [popAllC(k,1).Position, popAllC(k,2).Position] = ...
                        SBXCrossover(p1.Position, p2.Position);       
                elseif crossover_type == "gsbx"
                    grads_p1 = IK_find_grads(CostFunction, p1.Position, epsilon, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                    grads_p2 = IK_find_grads(CostFunction, p2.Position, epsilon, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                    [popAllC(k,1).Position, popAllC(k,2).Position] = ...
                        gSBXCrossover(p1.Position, p2.Position, grads_p1, grads_p2);        
                end
                
                
                popAllC(k,1).Position = max(popAllC(k,1).Position, VarMin);
                popAllC(k,1).Position = min(popAllC(k,1).Position, VarMax);
                
                popAllC(k,2).Position = max(popAllC(k,2).Position, VarMin);
                popAllC(k,2).Position = min(popAllC(k,2).Position, VarMax);
                
                % Evaluate the cost of the crossed individuals
                popAllC(k,1).Cost = CostFunction(popAllC(k,1).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                popAllC(k,2).Cost = CostFunction(popAllC(k,2).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                 
                % Compare solution to the best solution ever found
                if popAllC(k,1).Cost < BestSolution.Cost
                    BestSolution = popAllC(k,1);
                end
                
                if popAllC(k,2).Cost < BestSolution.Cost
                    BestSolution = popAllC(k,2);
                end
            end
            
            
            % Convert popc to single column matrix
            popAllC = popAllC(:);
            popAllM = popAllM(:);
            popAll = popAll(:);
            
            
            
            % Merge and Sort Populations
            pop = SortPopulation([popAll; popAllM; popAllC]);
            
            % Remove Extra Individuals
            popAll = pop(1:nPop);
            
            % Update Best Cost of Iterations
            BestCosts(it) = BestSolution.Cost;
            
            % Display Iteration Information
            if ShowIterInfo
                disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
            end
            
            % stock the search with desired tolerance is reached
            if BestCosts(it) <= DesiredTolerance
                robot = 'RRRRRR';
                DH = getDH_rad_cec(robot, BestSolution.Position, UnitChosen);
                T_eval_current = forwardKinematics_rad_cec(DH); 
                CurrentPose = getPose_rad_cec(T_eval_current, Dimension)'; 
                if ObjectiveType == "position"
                    if abs(DesiredPose(1)-CurrentPose(1)) < DesiredTolerance(1) && abs(DesiredPose(2)-CurrentPose(2)) < DesiredTolerance(1) && abs(DesiredPose(3)-CurrentPose(3)) < DesiredTolerance(1)
                        disp(['Solution found within ' num2str(it) ' iterations: Best Cost = ' num2str(BestCosts(it))]);
                        fprintf("Desired Pose Vector:")
                        DesiredPose  %#ok<NOPRT>                        
                        fprintf("Estimated Pose Vector:")
                        CurrentPose  %#ok<NOPRT>                                  
                        fprintf("Estimated Joint Vector:")
                        BestSolution.Position                                      
                        fprintf("Position Error Joint Vector (mm):")              
                        position_error = UnitApplied*sqrt(sum((DesiredPose(1:3)-CurrentPose(1:3)).^2)) %#ok<NOPRT>  
                        break
                    end
                else                
                    if abs(DesiredPose(1)-CurrentPose(1)) < DesiredTolerance(1) && abs(DesiredPose(2)-CurrentPose(2)) < DesiredTolerance(1) && abs(DesiredPose(3)-CurrentPose(3)) < DesiredTolerance(1) && abs(DesiredPose(4)-CurrentPose(4)) < DesiredTolerance(2) && abs(DesiredPose(5)-CurrentPose(5)) < DesiredTolerance(2) && abs(DesiredPose(6)-CurrentPose(6)) < DesiredTolerance(2)
                        disp(['Solution found within ' num2str(it) ' iterations: Best Cost = ' num2str(BestCosts(it))]);
                        fprintf("Desired Pose Vector:")
                        DesiredPose  %#ok<NOPRT>                        
                        fprintf("Estimated Pose Vector:")
                        CurrentPose  %#ok<NOPRT>                                  
                        fprintf("Estimated Joint Vector:")
                        BestSolution.Position                                      
                        fprintf("Position Error Joint Vector (mm):")              
                        position_error = UnitApplied*sqrt(sum((DesiredPose(1:3)-CurrentPose(1:3)).^2)) %#ok<NOPRT>  
                        fprintf("Orientation Error Joint Vector (degree):")         
                        orientation_error = rad2deg(sum(abs(DesiredPose(4:6)-CurrentPose(4:6)))/3) %#ok<NOPRT>
                        break
                    end
                end
            end
            
            
        end
        
       
        
        if it >= MaxIt
            robot = 'RRRRRR';
            DH = getDH_rad_cec(robot, BestSolution.Position, UnitChosen);
            T_eval_current = forwardKinematics_rad_cec(DH); 
            CurrentPose = getPose_rad_cec(T_eval_current, Dimension)'; 
            disp(['Maximum Number of iterations reached ' num2str(it) ' iterations: Best Cost = ' num2str(BestCosts(it))]);
            fprintf("Desired Pose Vector:")
            DesiredPose  %#ok<NOPRT>                        
            fprintf("Estimated Pose Vector:")
            CurrentPose  %#ok<NOPRT>                                   
            fprintf("Estimated Joint Vector:")
            CurrentJoints = BestSolution.Position  %#ok<NOPRT>                                        
            fprintf("Position Error Joint Vector (mm):")              
            position_error = UnitApplied*sqrt(sum((DesiredPose(1:3)-CurrentPose(1:3)).^2)) %#ok<NOPRT> 
            if ObjectiveType ~= "position"
                fprintf("Orientation Error Joint Vector (degree):")         
                orientation_error = rad2deg(sum(abs(DesiredPose(4:6)-CurrentPose(4:6)))/3) %#ok<NOPRT>
            end
        end
        
        % Results
        out.pop = pop;
        out.BestSolution = BestSolution;
        out.BestCosts = BestCosts;
        out.Iterations = it;
        if ObjectiveType == "position"
            out.PositionError = position_error;
        else
            out.PositionError = position_error;
            out.OrientationError = orientation_error;
        end
        
        
end
