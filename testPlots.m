for i=1:260
    joints(i,:) = out_gsbx_ga.pop(i).Position;
end

figure(1)
scatter3(joints(:,1),joints(:,2), joints(:,2), '.')
xlabel("\theta_{1}")
ylabel("\theta_{2}")
zlabel("\theta_{3}")
title("Joints 1,2,3")