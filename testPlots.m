for i=1:260
    joints(i,:) = out_gsbx_ga.pop(i).Position;
end

figure(1)
plot3(joints(:,1),joints(:,2), joints(:,2))
title("Joints 1,2,3")