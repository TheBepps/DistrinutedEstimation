% Define the time range
t = 0:0.1:100; % Time values from 0 to 100 with a step of 0.1

% Define parameters for the sinusoids
num_sin = 30; % Number of sinusoids
freq = linspace(0.005, 0.1, num_sin)*0.6; % Frequencies of the sinusoids
amp = rand(1, num_sin) *0.5; % Random amplitudes between 0 and 2
phase = rand(1, num_sin) * 2*pi; % Random phase shifts between 0 and 2*pi

% Initialize the hill profile
profile = zeros(size(t));

% Generate the hill profile by summing sinusoids
for i = 1:num_sin
    profile = profile + amp(i) * sin(2*pi*freq(i)*t + phase(i));
end

% Plot the profile
plot(t, profile, 'b');
xlabel('Distance');
ylabel('Height');
title('Profile Generated by Sinusoids with Random Amplitudes and Phases');
grid on;


% Define x-values for the shapes
anchors_x = [20, 50, 70, 15, 35, 99]; % X positions for the shapes

% Plot shapes using anchors_x array
hold on;
for i = 1:length(anchors_x)
    % Generate random color
    color = rand(1,3);
    plot(anchors_x(i), interp1(t, profile, anchors_x(i)), 'o', 'MarkerFaceColor', color, 'MarkerSize', 10);
end