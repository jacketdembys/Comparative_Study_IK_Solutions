function displayJoints(Q_current, robot)
    
    fprintf("\nCurrent joint configuration:")
    for c=1:length(Q_current)
            fprintf("\ntheta_%d = %f", c, Q_current(c));          
            fprintf("\nd_%d = %f", c, Q_current(c));
    end
    fprintf("\n")

end