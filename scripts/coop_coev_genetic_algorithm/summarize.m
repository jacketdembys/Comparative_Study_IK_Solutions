function [] = summarize(robot, x_val, unit_chosen, unit_applied, dimension, DesiredPose)
    
    DH = getDH_rad_cec(robot, x_val, unit_chosen);
    T_eval_current = forwardKinematics_rad_cec(DH); 
    
    fprintf("Desired Pose Vector:")
    DesiredPose %#ok<NOPRT>
    
    fprintf("Estimated Pose Vector:")
    CurrentPose = getPose_rad_cec(T_eval_current, dimension)' %#ok<NOPRT>
    
    fprintf("Estimated Joint Vector:")
    CurrentJoint = x_val %#ok<NOPRT>
    
    fprintf("Position Error (mm):")
    position_error = unit_applied*sqrt(sum((DesiredPose(1:3)-CurrentPose(1:3)).^2)) %#ok<NOPRT>  
    
    fprintf("Orientation Error (degree):")
    orientation_error = rad2deg(sum(abs(DesiredPose(4:6)-CurrentPose(4:6)))/3) %#ok<NOPRT>
            

end