%% Prepare workspare
clc, clear, close all

%% Plots the average position errors
x = linspace(3,180,60);
y = sin(x/2);

figure(1)
plot(x,y,"-","LineWidth",2,...
    "MarkerEdgeColor","blue","MarkerFaceColor",[0.65 0.85 0.90])
xlabel("Number of DoF")
ylabel("Average Position Errors (m)")
ax = gca; 
ax.FontSize = 20;
grid on
lgd = legend("NIK");
lgd.Location = 'northwest';



%% Plots the average position errors
x = linspace(3,180,60);
y = sin(x/2);

figure(2)
plot(x,y,"-","LineWidth",2,...
    "MarkerEdgeColor","blue","MarkerFaceColor",[0.65 0.85 0.90])
xlabel("Number of DoF")
ylabel("Average Iterations")
ax = gca; 
ax.FontSize = 20;
grid on
lgd = legend("NIK");
lgd.Location = 'northwest';



%% Plots the average runtime errors
x = linspace(3,180,60);
y = sin(x/2);

figure(3)
plot(x,y,"-","LineWidth",2,...
    "MarkerEdgeColor","blue","MarkerFaceColor",[0.65 0.85 0.90])
xlabel("Number of DoF")
ylabel("Average Computation Time (s)")
ax = gca; 
ax.FontSize = 20;
grid on
lgd = legend("NIK");
lgd.Location = 'northwest';

