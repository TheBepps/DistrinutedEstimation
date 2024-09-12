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

myMap.anchors_x = [5, 20, 40, 65, 80, 91];                  % Declare the anchor array
myMap.anchors_y = [5, 20, 40, 65, 80, 91];
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
ang_store = zeros(n, 2, length(time)); % actual orientation [th, phi]
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

% Predicted, estimated and measured position (Kalman filter)
pos_pred = zeros(3, length(time)); % predicted position (x,y,z)
pos_est = zeros(3, length(time)); % estimated position (x,y,z)
pos_meas = zeros(3, length(time)); % position measurements (x,y,z)
v_meas = zeros(3, length(time)); % velocity measurements (x,y,z)
acc_meas = zeros(3, length(time)); % acceleration measurements (x,y,z)

% First state measurement and estimated state initialization
pos_meas(:,2:3) = pos_store(1,:,2:3) + mvnrnd(mu_pos, sigma_pos)';
pos_est(:,2:3) = pos_meas(:,2:3);
x_init = pos_meas(1,3);
y_init = pos_meas(2,3);
z_init = pos_meas(3,3);
v_meas(1,2:3) = 0;
v_meas(2,2:3) = 0;
v_meas(3,2:3) = 0;
x_meas_prop = [x_init, x_init]; % [new_meas, old_meas] initialization
y_meas_prop = [y_init, y_init]; % [new_meas, old_meas] initialization
z_meas_prop = [z_init, z_init]; % [new_meas, old_meas] initialization

%

%% Consensus
A=[0 1 1 1 1 1;     % a(ij)
   1 0 0 0 1 1;
   1 0 0 1 0 1;
   1 0 1 0 1 0;
   1 1 0 1 0 0;
   1 1 1 0 0 0];

% Consensus relative position
delta_x=[0   0   0  -2.5  2.5 2.5]*2.5;  
delta_y=[0 -1.5 1.5   0  -1.5 1.5]*2.5;
delta_z=[0  1.5  0  -2.5 -1.5 1.5]*2.5;  % relative position between leader and followers [m]

% Uncertainty on followers relative distance measurements
mu_fol_dist = 0;
sigma_fol_dist = 1; % [m]

%

%% Run

for t=3:length(time)-2

    % Calculate attraction from anchor and compute leader velocity
    distance = norm(pos_store(1, :, t) - goal); 
    th = atan2(goal(2) - pos_store(1, 2, t), goal(1) - pos_store(1, 1, t));   % arctan(y/x)
%     phi = atan2(sqrt((goal(1) - pos_store(1, 1, t))^2 + (goal(2) - pos_store(1, 2, t))^2), goal(3) - pos_store(1, 3, t)); % arctan(sqrt(x^2 + y^2) / z)
    phi = acos((goal(3) - pos_store(1, 3, t)) / norm(goal - pos_store(1,:,t)));

    if distance > d_max
        distance = d_max;
    end
    
    vx = KL * distance * cos(th) * sin(phi);
    vy = KL * distance * sin(th) * sin(phi);
    vz = KL * distance * cos(phi);

    %% Calculate new leader velocity with obstacles

    % Calculate repulsion from obstacles
    repulsion = computeRepulsion3D([pos_store(1,1,t), pos_store(1,2,t), pos_store(1,3,t)], ob_pose(1:(length(ob_seabed) + length(ob_temp)), :), detect_R);
    
    % New leader velocity
    vx = vx + beta / 3 * repulsion(1);
    vy = vy + beta / 3 * repulsion(2);
    vz = vz + beta / 3 * repulsion(3); 
    
    % When the local minimum appears, add a random error
    if distance > 1 && abs(vx) <= 0.08 && abs(vy) <= 0.08 && abs(vz) <= 0.08 
        out(1) = -min(15, m / dt);  % go back along x direction (vx=-10->x=-1 if dt=0.1)
        out(2) = m * dt * (2 * m + rand(1)); % move +y randomly
        out(3) = m * dt * (2 * m + rand(1)); % Assuming random movement along z as well
        m = m + 1;
        disp("Trying to avoid a challenging obstacle...");
    else
        out = speedLimit3D([vel_store(1,1,t), vel_store(1,2,t), vel_store(1,3,t)], [vx, vy, vz], dt, max_vel, max_acc);
    end
    
    % Update pos_store and vel_store (real values) for leader
    vel_store(1,1,t+1) = out(1);                                        % vx
    vel_store(1,2,t+1) = out(2);                                        % vy
    vel_store(1,3,t+1) = out(3);                                        % vz
    pos_store(1,1,t+1) = pos_store(1,1,t) + dt * vel_store(1,1,t+1);    % x
    pos_store(1,2,t+1) = pos_store(1,2,t) + dt * vel_store(1,2,t+1);    % y
    pos_store(1,3,t+1) = pos_store(1,3,t) + dt * vel_store(1,3,t+1);    % z
    ang_store(1,1,t+1) = atan2(vel_store(1,2,t+1), vel_store(1,1,t+1)); % theta
    ang_store(1,2,t+1) = acos(vel_store(1,3,t+1)/sqrt( vel_store(1,1,t+1)^2+ vel_store(1,2,t+1)^2+ vel_store(1,3,t+1)^2));

    %% Calculate followers' velocity
    
    for i = 2:n  % starts from 2, leader is out
        sum_delta_x = 0;
        sum_delta_y = 0;
        sum_delta_z = 0; 
        for j = 1:n      % Linear Consensus control
                         % Condition Check: if A(i,j)==1: Checks if there is an edge between nodes i and j in the network represented by the adjacency matrix A.
                         % Edge Weight Calculation: Computes the edge weight q_ij based on Metroplis-Hastings weights
                         % Update Summations: Updates the sum of differences in x, y, and z coordinates weighted by q_ij.
            if A(i,j) == 1
                q_ij = 1 / (max(sum(A(i,:)), sum(A(j,:))) + 1); 
                ideal_distance = [delta_x(j)-delta_x(i), delta_y(j)-delta_y(i), delta_z(j)-delta_z(i)];
                actual_distance = pos_store(j,:,t) - pos_store(i,:,t);
                meas_distance = actual_distance + mvnrnd(mu_fol_dist, sigma_fol_dist)'; % Assuming mvnrnd returns a vector with three elements
                % Accumulate contributes
                sum_delta_x = sum_delta_x + q_ij * (meas_distance(1) - ideal_distance(1));                   
                sum_delta_y = sum_delta_y + q_ij * (meas_distance(2) - ideal_distance(2));
                sum_delta_z = sum_delta_z + q_ij * (meas_distance(3) - ideal_distance(3)); % Update sum_delta_z
            end
        end
    
        % Ideal velocity without repulsion from obstacles
        distance = sqrt(sum_delta_x^2 + sum_delta_y^2 + sum_delta_z^2); 
        th = atan2(sum_delta_y, sum_delta_x); % Calculate angle in the xy-plane
        phi = atan2(sqrt(sum_delta_x^2 + sum_delta_y^2), sum_delta_z); % Calculate angle with the z-axis
        if distance > d_max
            distance = d_max;
        end
        % Update velocities in x, y, and z directions
        vel_store(i, 1, t+1) = vel_store(1, 1, t) + KF * distance * cos(th) * sin(phi); 
        vel_store(i, 2, t+1) = vel_store(1, 2, t) + KF * distance * sin(th) * sin(phi); 
        vel_store(i, 3, t+1) = vel_store(1, 3, t) + KF * distance * cos(phi); 
        % Consider repulsion among submarines and store their pose to obs_pose
        kk = 0;
        obs_pose = zeros(fol_num,2);
        for j=1:n
            if j~=i
                kk = kk+1;
                obs_pose(kk,1) = pos_store(j,1,t);
                obs_pose(kk,2) = pos_store(j,2,t);
                obs_pose(kk,3) = pos_store(j,3,t);
            end
        end
        ob_pose = [ob_temp; ob_seabed; obs_pose];
        repulsion = computeRepulsion3D([pos_store(i,1,t), pos_store(i,2,t), pos_store(i,3,t)], ob_pose, detect_R);
        % New follower velocity
        vel_store(i,:,t+1) = vel_store(i,:,t+1) + beta*repulsion;
        
        % Update followers position and velocity
        out = speedLimit3D(vel_store(i, :, t), vel_store(i, :, t+1), dt, max_vel, max_acc);
        vel_store(i, 1, t+1) = out(1);
        vel_store(i, 2, t+1) = out(2);
        vel_store(i, 3, t+1) = out(3); 
        pos_store(i, 1, t+1) = pos_store(i, 1, t) + dt * vel_store(i, 1, t+1);
        pos_store(i, 2, t+1) = pos_store(i, 2, t) + dt * vel_store(i, 2, t+1);
        pos_store(i, 3, t+1) = pos_store(i, 3, t) + dt * vel_store(i, 3, t+1);
        ang_store(i,1,t+1) = atan2(norm([vel_store(i,2,t+1), vel_store(i,3,t+1)]), vel_store(i,1,t+1)); % theta
        ang_store(i,2,t+1) = acos(vel_store(i,3,t+1)/sqrt( vel_store(i,1,t+1)^2+ vel_store(i,2,t+1)^2+ vel_store(i,3,t+1)^2));
    end
    
    %% Reaching goal

    now = [pos_store(1,1,t+1),pos_store(1,2,t+1),pos_store(1,3,t+1)];
    if norm(now-goal) < 1
        if anchor_num == length(myMap.anchors_x) && fin_goal_reached == 0
            disp('Final goal has been reached');
            fin_goal_reached = 1;
        end

        if anchor_num < length(myMap.anchors_x) && not(anchor_num == 1)
            string = ['Anchor number ', num2str(anchor_num-1), ' has been reached'];
            disp(string);
            anchor_num  = anchor_num + 1;
        end

        if anchor_num == 1
            disp('Starting from anchor number 0');
            anchor_num  = anchor_num + 1;
        end

        if fin_goal_reached
            stop=0;
            for i=2:n      % check if every submarine reaches final position ("zero" velocity)
                if abs(vel_store(i,1,t))<=0.5*sigma_fol_dist && abs(vel_store(i,2,t))<=0.5*sigma_fol_dist && abs(vel_store(i,3,t))<=0.5*sigma_fol_dist
                    stop=1;
                else
                    stop=0;
                    break;  % someone moves so we don't need to check others
                end
            end
            if stop
                disp('Everyone has reached the final position');
                break;      % end for loop "Run" section               
            end
        end
        if anchor_num < length(myMap.anchors_x)+1
            m=1;    % reset local minima variable
            goal = [myMap.anchors_x(anchor_num) myMap.anchors_y(anchor_num) (myMap.anchors_z(anchor_num)+myMap.anchor_offset)];
        end
    end




    






    % Plot the 3D surface
    arrow_length=0.2;
    figure(1);
    hold off;   
    drawMap(myMap);
    %plot's view properties
    view_props = [max(min(min(pos_store(:,1,t)))-20,0)   
                min(max(max(pos_store(:,1,t)))+20,100)     
                max(min(min(pos_store(:,2,t)))-20,0)   
                min(max(max(pos_store(:,2,t)))+20,100)   
%                 min(min(myMap.profile(max(floor(min(min(pos_store(:,1,t))))-20,1):min(floor(max(max(pos_store(:,1,t))))+20,100), max(floor(min(min(pos_store(:,2,t))))-20,1):min(floor(max(max(pos_store(:,2,t))))+20,100))))    
                -55;
                max(max(pos_store(:,3,t)))+10];
    axis(view_props);
    if size(ob_temp)~=[0 0]
            plot3(ob_temp(:,1),ob_temp(:,2),ob_temp(:,3),'r.','MarkerSize',20);
            hold on;
    end
    plot3(squeeze(pos_store(1,1,2:t+1)),squeeze(pos_store(1,2,2:t+1)),squeeze(pos_store(1,3,2:t+1)),'b','LineWidth',0.5); % plot path
%     hold on;
%     quiver3(pos_store(:,1,t+1),pos_store(:,2,t+1),pos_store(:,3,t+1),arrow_length*cos(ang_store(:,1,t+1)).*sin(ang_store(:,2,t+1)),arrow_length*sin(ang_store(:,1,t+1)).*sin(ang_store(:,2,t+1)),arrow_length*cos(ang_store(:,2,t+1)),'k');
    quiver3(pos_store(:,1,t+1),pos_store(:,2,t+1),pos_store(:,3,t+1),arrow_length*vel_store(:,1,t+1),arrow_length*vel_store(:,2,t+1),arrow_length*vel_store(:,3,t+1),'k');
    %     hold on;
    for j=1:n
        if j==1    % drawing leader in a different colour
            plotTriangle3D(pos_store(j,1,t+1), pos_store(j,2,t+1), pos_store(j,3,t+1), ang_store(j,1,t+1), ang_store(j,2,t+1),'r');
        else
            plotTriangle3D(pos_store(j,1,t+1), pos_store(j,2,t+1), pos_store(j,3,t+1), ang_store(j,1,t+1), ang_store(j,2,t+1),'g');
        end        
    end
    hold on;
    drawnow;
end
