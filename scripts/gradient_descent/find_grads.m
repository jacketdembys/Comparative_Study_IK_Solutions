function grads = find_grads(fmin, x0, epsilon)

    % https://www.reddit.com/r/MachineLearning/comments/2e8797/gradient_descent_without_a_derivative/

    num_total = length(x0);
    grads = zeros(num_total,1);
    for i=1:num_total
        x = x0; 
        x(i) = x(i) + epsilon(i);
        grads(i) = (fmin(x) - fmin(x0))/epsilon(i);        
    end

end