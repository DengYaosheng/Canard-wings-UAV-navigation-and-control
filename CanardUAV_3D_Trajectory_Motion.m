% Initial plot
figure;
plot3(X_OUTPUT, Y_OUTPUT, Z_OUTPUT, 'b-'); % Path
hold on;
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('3D Figure-Eight Trajectory');
grid on;

% Load and create the drone model
global Quad;
load Quad_plotting_model;  % Load model data
plot_quad_model();  % Initialize the drone body

% Check if Quad is properly initialized
if ~isfield(Quad, 'X_arm') || ~isvalid(Quad.X_arm)
    error('Quad model is not properly initialized.');
end

% Loop through each point
for k = 1:length(X_OUTPUT)
    % Check if the figure still exists
    if ~ishandle(Quad.X_arm)
        warning('Figure has been closed. Stopping animation.');
        break;
    end

    updateDrone(X_OUTPUT(k), Y_OUTPUT(k), Z_OUTPUT(k));
    drawnow;
    pause(0.01);
end

hold off;


% Function to initialize the drone model
function plot_quad_model()
    global Quad;
    % Assuming Quad structure is loaded and contains necessary fields
    Quad.X_arm = patch('xdata', Quad.X_armX, 'ydata', Quad.X_armY, 'zdata', Quad.X_armZ, 'facealpha', .9, 'facecolor', 'b');
    Quad.Y_arm = patch('xdata',Quad.Y_armX,'ydata',Quad.Y_armY,'zdata',Quad.Y_armZ,'facealpha',.9,'facecolor','b');
    Quad.Motor1 = patch('xdata',Quad.Motor1X,'ydata',Quad.Motor1Y,'zdata',Quad.Motor1Z,'facealpha',.3,'facecolor','g');
    Quad.Motor2 = patch('xdata',Quad.Motor2X,'ydata',Quad.Motor2Y,'zdata',Quad.Motor2Z,'facealpha',.3,'facecolor','k');
    Quad.Motor3 = patch('xdata',Quad.Motor3X,'ydata',Quad.Motor3Y,'zdata',Quad.Motor3Z,'facealpha',.3,'facecolor','k');
    Quad.Motor4 = patch('xdata',Quad.Motor4X,'ydata',Quad.Motor4Y,'zdata',Quad.Motor4Z,'facealpha',.3,'facecolor','k');

end

function updateDrone(x, y, z)
    global Quad;

    % Update the position of each component of the drone
    % X Arm
    set(Quad.X_arm, 'xdata', x + Quad.X_armX, 'ydata', y + Quad.X_armY, 'zdata', z + Quad.X_armZ);
    set(Quad.Y_arm, 'xdata', x + Quad.Y_armX, 'ydata', y + Quad.Y_armY, 'zdata', z + Quad.Y_armZ);
    set(Quad.Motor1, 'xdata', x + Quad.Motor1X, 'ydata', y + Quad.Motor1Y, 'zdata', z + Quad.Motor1Z);
    set(Quad.Motor2, 'xdata', x + Quad.Motor2X, 'ydata', y + Quad.Motor2Y, 'zdata', z + Quad.Motor2Z);
    set(Quad.Motor3, 'xdata', x + Quad.Motor3X, 'ydata', y + Quad.Motor3Y, 'zdata', z + Quad.Motor3Z);
    set(Quad.Motor4, 'xdata', x + Quad.Motor4X, 'ydata', y + Quad.Motor4Y, 'zdata', z + Quad.Motor4Z);
end
