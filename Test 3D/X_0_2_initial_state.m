clc;
clear;
%addpath('C:\Users\Arianna\Desktop\DistributedEstimation\Test 2D')

% Time
dt = 0.18;
time = 0:dt:150;

%% Parameters
d_max = 2;          % above d_max, velocity remains constant 
detect_R = 4;       % distance to detect obstacles
ob_num = 25;        % obstacles number
beta = 14;          % object repulsion parameter
m = 1;              % local minima detection
KL = 1;             % leader's velocity coeff. KL<KF in order to "speed up" followers' movements
KF = 1.2;           % followers' velocity coeff. Be aware to increase more than 1, velocity & obstacles' repulsion are correlated
max_vel = 6;        % maximum velocity
max_acc = 3;        % maximum acceleration
fol_num = 5;        % number of followers
n = fol_num+1;      % fol_num followers and 1 leader
%%

%% Define the structure of MAP
myMap = struct();

myMap.dim = 100;                                            % Map dimension
myMap.k = 0:0.5:myMap.dim;                                  % Time values from 0 to max dimension with a step of 0.1
myMap.profile = zeros(size(myMap.k,2));                     % Initialize the seabed profile
myMap.x_profile=zeros(size(myMap.k));
myMap.y_profile=zeros(size(myMap.k));
myMap.num_sin = 20;                                         % number of sinusoids
myMap.amp_x = rand(1, myMap.num_sin)*2;                     % Random amplitudes between 0 and 2
myMap.amp_y = rand(1, myMap.num_sin)*2;                 
myMap.phase_x = rand(1, myMap.num_sin)* 2*pi;               % Random phase shifts between 0 and 2*pi
myMap.phase_y = rand(1, myMap.num_sin)* 2*pi;          
myMap.freq_x = linspace(0.005, 0.03, myMap.num_sin)*0.6;    % Frequencies of the sinusoids
myMap.freq_y = linspace(0.005, 0.03, myMap.num_sin)*0.6; 
myMap.ground_zero = 40;                                     % Zero level of the sea

myMap.anchors_x = [5, 20, 40, 65, 80, 95];                  % Declare the anchor array
myMap.anchors_y = [5, 20, 40, 65, 80, 95];
myMap.anchors_z = zeros(size(myMap.anchors_x));

myMap.anchor_offset=15;

for i = 1:myMap.num_sin
    myMap.x_profile = myMap.x_profile + myMap.amp_x(i) * sin(2*pi*myMap.freq_x(i)*myMap.k + myMap.phase_x(i));
    myMap.y_profile = myMap.y_profile + myMap.amp_y(i) * sin(2*pi*myMap.freq_y(i)*myMap.k + myMap.phase_y(i));
end

myMap.profile = myMap.x_profile' + myMap.y_profile;
myMap.profile = myMap.profile - myMap.ground_zero;          % Set level zero


% anchors_y
for i = 1:length(myMap.anchors_x)
    myMap.anchors_z(i) = interp2(myMap.k, myMap.k, myMap.profile, myMap.anchors_x(i), myMap.anchors_y(i));
end

% Obstacles
ob_temp=obstBuild3D(myMap, ob_num, detect_R);

% Create ob_seabed as avoidance objects
sampled_k = floor(myMap.k(detect_R:detect_R:end));
ob_seabed=zeros(numel(sampled_k)^2,3);
for i=1:numel(sampled_k)
    for j=1:numel(sampled_k)
        ob_seabed(m,:) = [sampled_k(i), sampled_k(j),  myMap.profile(sampled_k(j)*detect_R/2,sampled_k(i)*detect_R/2)];
    m=m+1;
    end
end
m=1;

% Concatenate ob_temp with the seabed
ob_pose = [ob_temp; ob_seabed];

% First goal
goal = [myMap.anchors_x(1), myMap.anchors_y(1), 0];
anchor_num = 1;

fin_goal_reached = 0;

%% Initial state

pos_store = zeros(n, 3, length(time)); % actual states [x, y, z]
ang_store = zeros(n, 3, length(time)); % actual orientation [roll, pitch, yaw]
vel_store = zeros(n, 3, length(time)); % actual inputs [vx, vy]

% Initial position 
p_0 = zeros(n,3); % [x, y, z] for n submarines
p_0(1,1:3) = [myMap.anchors_x(1), myMap.anchors_y(1), 0]; %leader's first position known
for i=2:n
    p_0(i,1) = abs(10*rand(1));
    p_0(i,2) = abs(15*rand(1));
    p_0(i,3) = -2-detect_R/2*i*rand(1);
end

% Initial velocity
v_0 = zeros(n,3); % [vx, vy, vz] for for n submarines

% Define triangles initial state 
pos_store(:,:,2:3) = cat(3, p_0, p_0); % starting from time=2
vel_store(:,:,2:3) = cat(3, v_0, v_0); % starting from time=2

%% Uncertainty

% Velocity (input) measurements
mu_vel = 0;
sigma_vel = 0.2;

%

% Proprioceptive sensor
mu_acc = 0.05; 
sigma_acc = 0.1;
H = eye(3);
Prob_sensor = 0.25; % new measurements provided every 4 steps (on average)

%

% Exteroceptive sensors
mu_pos = 0; 
sigma_pos = 0.1; 
accuracy_ratio = 3; % how much ext meas are considered to be more reliable than proprioceptive meas

%

% leader is initially on the first anchor: exteroceptive meas activated
ext_range = 5; % if distance between a submarine and the anchor is less than ext_range, ext meas are activated
ext_meas = [1, 1]; % [value at current time instant, value at previous time instant] (ext meas initially activated)
flag_leader_close_to_anchor = [1,1]; % [value at current time instant, value at previous time instant] (leader starts on anchor)

%

%% Consensus
A=[0 1 1 1 1 1;     % a(ij)
   1 0 0 0 1 1;
   1 0 0 1 0 1;
   1 0 1 0 1 0;
   1 1 0 1 0 0;
   1 1 1 0 0 0];

delta_x=[0   0   0  -2.5  2.5 2.5]*2.5;  
delta_y=[0 -1.5 1.5   0  -1.5 1.5]*2.5;
delta_z=[0  1.5  0  -2.5 -1.5 1.5]*2.5;  % relative position between leader and followers [m]







% Plot the 3D surface
figure;
drawMap(myMap);
if size(ob_temp)~=[0 0]
        plot3(ob_temp(:,1),ob_temp(:,2),ob_temp(:,3),'Xy','LineWidth',2);
        hold on;
end
for j=1:n
    hold on;
    if j==1    % drawing leader in a different colour
        plotTriangle3D(pos_store(j,1,2), pos_store(j,2,2), pos_store(j,3,2), ang_store(j,1,2), ang_store(j,2,2), ang_store(j,3,2),'r');
    else
        plotTriangle3D(pos_store(j,1,2), pos_store(j,2,2), pos_store(j,3,2), ang_store(j,1,2), ang_store(j,2,2), ang_store(j,3,2),'g');
    end        
end
