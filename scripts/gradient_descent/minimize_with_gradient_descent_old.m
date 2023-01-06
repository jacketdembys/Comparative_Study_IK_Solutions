function [recap_results, fval] = minimize_with_gradient_descent(fmin, x, max_steps, learning_rate, epsilon)

    recap_results = zeros(max_steps, 1);
    
    %% max_steps is equivalent to the number of generations
    for i=1:max_steps
        grads = find_grads(fmin, x, epsilon);
        x = x - learning_rate*grads;
        f = fmin(x);
        recap_results(i) = f;
    end

    fval = f;
    
end