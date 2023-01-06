function grads = IK_find_grads(fmin, x0, epsilon, desired_pose, U, d, ot)

    % https://www.reddit.com/r/MachineLearning/comments/2e8797/gradient_descent_without_a_derivative/

    num_total = length(x0);
    grads = zeros(num_total,1);
    for i=1:num_total
        x = x0; 
        x(i) = x(i) + epsilon(i);
        grads(i) = (fmin(x, desired_pose, U, d, ot) - fmin(x0, desired_pose, U, d, ot))/epsilon(i);        
    end

end