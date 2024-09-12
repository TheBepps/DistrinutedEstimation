clc;
clear;

%map dimension
m_dim=150;

% Declare the anchor array
anchors_x = [20, 40, 70];

% Call seabedInitialization function with the anchor array
num_sin = 30; % Number of sinusoids
amp=rand(1, num_sin)*2; % Random amplitudes between 0 and 2
phase=rand(1, num_sin)* 2*pi; % Random phase shifts between 0 and 2*pi
[t, profile] = seabedInitialization_2d(anchors_x,amp,phase,num_sin,m_dim);

% Define triangle dynamics [x, y, theta]
triangle_dynamics = [0, 20, pi/2]; % Initial dynamics

% Define the number of steps and the duration of each step
num_steps = 100;
duration = 0.05; % Adjust as needed

% Determine the angle between the initial orientation and the direction towards the first anchor point
direction_theta = atan2(interp1(t, profile, anchors_x(1)) - triangle_dynamics(2), anchors_x(1) - triangle_dynamics(1));
% Calculate the angle difference (change in theta)
angle_difference = direction_theta - triangle_dynamics(3);
% Ensure the angle difference is between -pi and pi for proper rotation
angle_difference = mod(angle_difference + pi, 2*pi) - pi;
% Calculate the step size for theta
step_size_theta = angle_difference / num_steps;

% Calculate the step size for x and y coordinates
step_size_x = (anchors_x(1) - triangle_dynamics(1)) / num_steps;
step_size_y = (interp1(t, profile, anchors_x(1)) - triangle_dynamics(2)) / num_steps;

% Loop through each step
for step = 1:num_steps
    % Update x and y coordinates
    triangle_dynamics(1) = triangle_dynamics(1) + step_size_x;
    triangle_dynamics(2) = triangle_dynamics(2) + step_size_y;
    triangle_dynamics(3) = triangle_dynamics(3) + step_size_theta;
    
    % Clear the previous triangle
    clf;
    
    % Call seabedInizialization function with the anchor array
    seabedInitialization_2d(anchors_x,amp,phase,num_sin,m_dim);
    
    % Plot the triangle with dynamics
    plotTriangle2d(triangle_dynamics(1), triangle_dynamics(2), triangle_dynamics(3));
    
    % Pause for a short duration to create animation effect
    pause(duration);
end
