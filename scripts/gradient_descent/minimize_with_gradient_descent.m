function [out] = minimize_with_gradient_descent(problem, params)

    fmin = problem.CostFunction;
    n = problem.nVar; 
    
    max_steps = params.max_steps;
    learning_rate = params.learning_rate;
    epsilon = params.epsilon;    
    lb = params.lb;
    ub = params.ub;
    ShowIterInfo = params.ShowIterInfo;
    
    x = (ub-lb).*rand(n,1) + lb;

    recap_results = zeros(max_steps, 1);
    
    %% max_steps is equivalent to the number of generations
    for i=1:max_steps
        grads = find_grads(fmin, x, epsilon);
        x = x - learning_rate*grads;
        f = fmin(x);
        recap_results(i) = f;
        
        % Display Iteration Information
        if ShowIterInfo
            disp(['Iteration ' num2str(i) ': Best Cost = ' num2str(recap_results(i))]);
        end
            
        
    end

    fval = f;
    out.BestCosts = recap_results;
    out.BestSolution.Position = x;
    out.BestSolution.Cost = f;
    
end