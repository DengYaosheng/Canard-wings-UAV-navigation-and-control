% Assuming x_total, y_total, z_total are 1x1 double timeseries objects
% Extract data from timeseries objects
X_OUTPUT2 = x_total.Data;
Y_OUTPUT2 = y_total.Data;
Z_OUTPUT2 = z_total.Data;


% Assuming X_OUTPUT, Y_OUTPUT, Z_OUTPUT are already defined and are numeric arrays

% Create the initial plot
figure;
plot3(X_OUTPUT, Y_OUTPUT, Z_OUTPUT,"LineWidth",2); % The first path in blue
hold on; % Keep the plot
plot3(X_OUTPUT2, Y_OUTPUT2, Z_OUTPUT2, "LineWidth",2); % The second path in green

dronePlot1 = plot3(X_OUTPUT(1), Y_OUTPUT(1), Z_OUTPUT(1), 'ro', 'MarkerSize', 10); % Initial position of first drone
dronePlot2 = plot3(X_OUTPUT2(1), Y_OUTPUT2(1), Z_OUTPUT2(1), 'mo', 'MarkerSize', 10); % Initial position of second drone

xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('3D Dual Trajectories');
grid on;

hold off;
