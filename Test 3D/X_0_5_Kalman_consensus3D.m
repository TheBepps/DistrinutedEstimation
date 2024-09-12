clc;
clear;
% addpath('C:\Users\Arianna\Desktop\DistributedEstimation\Test 3D')

% Time
dt = 0.15;
time = 0:dt:150;

%% Parameters
d_max = 2;          % above d_max, velocity remains constant 
detect_R = 4;       % distance to detect obstacles
ob_num = 25;        % obstacles number
beta = 14;          % object repulsion parameter
m = 1;              % local minima detection
KL = 2;             % leader's velocity coeff. KL<KF in order to "speed up" followers' movements
KF = 2.3;           % followers' velocity coeff. Be aware to increase more than 1, velocity & obstacles' repulsion are correlated
max_vel = 4;        % maximum velocity
max_acc = 4;        % maximum acceleration
fol_num = 7;        % number of followers
n = fol_num+1;      % fol_num followers and 1 leader
%% Model

A_model = eye(3); % x, y, z
B_model = eye(3)*dt; % vx, vy, vz

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

myMap.anchors_x = [6, 20, 40, 60, 75, 87];                  % Declare the anchor array
myMap.anchors_z = zeros(size(myMap.anchors_x));

myMap.anchor_offset=15;

for i = 1:myMap.num_sin
    myMap.x_profile = myMap.x_profile + myMap.amp_x(i) * sin(2*pi*myMap.freq_x(i)*myMap.k + myMap.phase_x(i));
    myMap.y_profile = myMap.y_profile + myMap.amp_y(i) * sin(2*pi*myMap.freq_y(i)*myMap.k + myMap.phase_y(i));
end

myMap.profile = myMap.x_profile' + myMap.y_profile;
myMap.profile = myMap.profile - myMap.ground_zero;          % Set level zero

% anchors_z
for i = 1:length(myMap.anchors_x)
    myMap.anchors_y(i) = randi([max(myMap.anchors_x(i)-10,1),min(myMap.anchors_x(i)+10,100)]);
    myMap.anchors_z(i) = interp2(myMap.k, myMap.k, myMap.profile, myMap.anchors_x(i), myMap.anchors_y(i));
end

% Obstacles
ob_temp=obstBuild3d(myMap, ob_num, detect_R);

% Create ob_seabed as avoidance objects
sampled_k = round(myMap.k(detect_R*0.5:detect_R*0.5:end));
ob_seabed = zeros(numel(sampled_k), 3, numel(sampled_k)); % Initialize to 50x3x50
ob_seabed_short=ob_seabed;
for i=1:numel(sampled_k)
    for j=1:numel(sampled_k)
        ob_seabed(j,:,i) = [sampled_k(i), sampled_k(j),  myMap.profile(sampled_k(j)*2,sampled_k(i)*2)];
    end
end

% Concatenate ob_temp with the seabed, reshaped from 50x3x50 to 2500x3
ob_pose = [ob_temp; zeros(fol_num,3); reshape(permute(ob_seabed, [1 3 2]), [], 3)];

% First goal
goal = [myMap.anchors_x(1), myMap.anchors_y(1), 0]; % on sea level (starting from the surface)
anchor_num = 1;

fin_goal_reached = 0;

%% Initial state

pos_store = zeros(n, 3, length(time)); % actual states [x, y, z]
ang_store = zeros(n, 2, length(time)); % actual orientation [yaw, pitch] (roll not considered)
vel_store = zeros(n, 3, length(time)); % actual inputs [vx, vy, vz]

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

% Process noise covariance matrix
 Q_cov = [0.15, 0.17, 0.23; 
          0.17, 0.20, 0.22;
          0.23, 0.22, 0.25];

% Process noise gain matrix
G = eye(3);

% Position (state) measurements

% Proprioceptive sensor
mu_acc = 0.05;
sigma_acc = 0.1;
H = eye(3);
Prob_sensor = 0.25; % new measurements provided every 4 steps (on average)

% Proprioceptive measurement noise covariance matrix
R_cov_acc = [0.43, 0.37, 0.35; 
             0.37, 0.24, 0.26;
             0.35, 0.26, 0.30];

% Exteroceptive sensors
mu_pos = 0; 
sigma_pos = 0.1; 
accuracy_ratio = 3; % how much ext meas are considered to be more reliable than proprioceptive meas

% Exteroceptive measurement noise covariance matrix
R_cov_pos = [0.28, 0.25, 0.24; 
             0.25, 0.26, 0.23; 
             0.24, 0.23, 0.22];

% leader is initially on the first anchor: exteroceptive meas activated
ext_range = 5; % if distance between a submarine and the anchor is less than ext_range, ext meas are activated
ext_meas = [1, 1]; % [value at current time instant, value at previous time instant] (ext meas initially activated)
flag_leader_close_to_anchor = [1,1]; % [value at current time instant, value at previous time instant] (leader starts on anchor)

% Predicted, estimated and measured position (Kalman filter)
pos_pred = zeros(3, length(time));  % predicted position (x,y,z)
pos_est = zeros(3, length(time));   % estimated position (x,y,z)
pos_meas = zeros(3, length(time));  % position measurements (x,y,z)
v_meas = zeros(3, length(time));    % velocity measurements (x,y,z)
acc_meas = zeros(3, length(time));  % acceleration measurements (x,y,z)

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

% Error covariance matrix of state estimate
P_init = 5*eye(3); 
P_store = zeros(3, 3, length(time));
P_store(:,:,2:3) = cat(3, P_init, P_init); % Initialization
P_pred = zeros(3, 3, length(time));

%% Consensus
A=[0 1 1 1 1 1 1 1;     % a(ij)
   1 0 0 0 1 1 0 0;
   1 0 0 1 1 1 0 0;
   1 0 1 0 1 0 0 0;
   1 1 1 1 0 0 0 0;
   1 1 1 0 0 0 1 1;
   1 0 0 0 0 1 0 1;
   1 0 0 0 0 1 1 0];

% Consensus relative position
delta_x=[0   3  1.5   0   1.5  -1    0   -2 ]*3;
delta_y=[0   3   0   1.5  1.5  -1   -2    0 ]*3;  
delta_z=[0   0  1.5  1.5 -1.5   1  -1.5 -1.5]*3;  % relative position between leader and followers [m]

% Uncertainty on followers relative distance measurements
mu_fol_dist = 0;
sigma_fol_dist = 1; % [m]

% Followers measurement noise covariance matrix
R_cov_fol = [0.42, 0.50, 0.45; 
             0.50, 0.99, 0.70; 
             0.45, 0.70, 0.52];

%% Run

for t=3:length(time)-2

    % Calculate attraction from anchor and compute leader velocity
    distance = norm(pos_est(:, t)' - goal); % pos_est(t) because leader doesn't know real position (pos_store(t))
    yaw = atan2(goal(2) - pos_est(2, t), goal(1) - pos_est(1, t));   % arctan(y/x)
    pitch = acos((goal(3) - pos_est(3, t)) / norm(goal - pos_est(:,t)'));

    if distance > d_max
        distance = d_max;
    end
    
    vx = KL * distance * cos(yaw) * sin(pitch);
    vy = KL * distance * sin(yaw) * sin(pitch);
    vz = KL * distance * cos(pitch);

    %% Calculate new leader velocity with obstacles
    
    repulsion = computeRepulsion3D([pos_store(1,1,t), pos_store(1,2,t), pos_store(1,3,t)], ob_pose(1:length(ob_temp), :), detect_R);
    repulsion = repulsion + computeRepulsion3D([pos_store(1,1,t), pos_store(1,2,t), pos_store(1,3,t)], ob_pose(length(ob_temp)+fol_num:end, :), detect_R*2);
    
    % New leader velocity
    vx = vx + beta / 3 * repulsion(1);
    vy = vy + beta / 3 * repulsion(2);
    vz = vz + beta / 3 * repulsion(3); 
    
    % When the local minimum appears, add a random error
    if distance > 1 && abs(vx) <= 0.08 && abs(vy) <= 0.08 && abs(vz) <= 0.08 
        out(1) = -min(15, m / dt);  % go back along x direction (vx=-10->x=-1 if dt=0.1)
        out(2) = -min(15, m / dt); % go back along y direction 
        out(3) = m * dt * (2 * m + rand(1)); % random movement along +z 
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
    ang_store(1,1,t+1) = atan2(vel_store(1,2,t+1), vel_store(1,1,t+1)); % yaw
    ang_store(1,2,t+1) = acos(vel_store(1,3,t+1)/sqrt( vel_store(1,1,t+1)^2+ vel_store(1,2,t+1)^2+ vel_store(1,3,t+1)^2)); % pitch

    %% Measurements

    % New acc measurement (proprioceptive sensor)
    real_acc = [(vel_store(1,1,t+1)-vel_store(1,1,t))/dt, (vel_store(1,2,t+1)-vel_store(1,2,t))/dt, (vel_store(1,3,t+1)-vel_store(1,3,t))/dt]; % real_acc
    acc_meas(:,t+1) = real_acc + mvnrnd(mu_acc, sigma_acc)'; % real acc + measurement error
    % First integration
    v_meas(1,t+1) = runge_kutta_4(acc_meas(1,t), acc_meas(1,t+1), dt, v_meas(1,t));
    v_meas(2,t+1) = runge_kutta_4(acc_meas(2,t), acc_meas(2,t+1), dt, v_meas(2,t));
    v_meas(3,t+1) = runge_kutta_4(acc_meas(3,t), acc_meas(3,t+1), dt, v_meas(3,t));
    % Second integration
    x_meas_prop(1) = runge_kutta_4(v_meas(1,t), v_meas(1,t+1), dt, x_meas_prop(2));
    y_meas_prop(1) = runge_kutta_4(v_meas(2,t), v_meas(2,t+1), dt, y_meas_prop(2));
    z_meas_prop(1) = runge_kutta_4(v_meas(3,t), v_meas(3,t+1), dt, z_meas_prop(2));
    
    % Store results
    x_meas = x_meas_prop(1);
    y_meas = y_meas_prop(1);
    z_meas = z_meas_prop(1);

    % When a follower is close to the anchor, exteroceptive measurements
    % are activated and fused together with proprioceptive ones.
    if (ext_meas(1) == 1 || ext_meas(2) == 1) && (flag_leader_close_to_anchor(1) == 0 && flag_leader_close_to_anchor(2) == 0) % only followers are close to the anchor
        ext_meas = pos_store(1,:,t+1) + mvnrnd(mu_pos, sigma_pos)';
        % Sensor fusion
        x_meas = x_meas_prop(1)/(accuracy_ratio*accuracy_ratio + 1) + ext_meas(1)/(1 + 1/(accuracy_ratio*accuracy_ratio)); % x
        y_meas = y_meas_prop(1)/(accuracy_ratio*accuracy_ratio + 1) + ext_meas(2)/(1 + 1/(accuracy_ratio*accuracy_ratio)); % y
        z_meas = z_meas_prop(1)/(accuracy_ratio*accuracy_ratio + 1) + ext_meas(3)/(1 + 1/(accuracy_ratio*accuracy_ratio)); % z
    end
        
    % If the leader is close to an anchor, it can measure more
    % accurately its position. In this case it completely relies on exteroceptive measurements.
    if flag_leader_close_to_anchor(1) == 1 || flag_leader_close_to_anchor(2) == 1 % leader can see the anchor
        ext_meas = pos_store(1,:,t+1) + mvnrnd(mu_pos, sigma_pos)';
        x_meas = ext_meas(1);
        y_meas = ext_meas(2);
        z_meas = ext_meas(3);
    end

    % Store measurements to be considered by the KF
    pos_meas(:,t+1) = [x_meas; y_meas; z_meas];

    % Store initial values for the next integration step (prop meas)
    x_meas_prop(2) = x_meas_prop(1);
    y_meas_prop(2) = y_meas_prop(1);
    z_meas_prop(2) = z_meas_prop(1);

    %% Update estimated state with Kalman filter (leader)

    % Input error
    v_bar = vel_store(1,:,t+1) + mvnrnd(mu_vel, sigma_vel)'; % v_bar(t+1)

    % PREDICTION STEP
    pos_pred(:,t+1) = A_model*pos_est(:,t) + B_model*v_bar'; % p_est(t) + dt*v_bar
    P_pred(:,:,t+1) = A_model*P_store(:,:,t)*A_model' + G*Q_cov*G'; % A*P(t)*A' + G*Q*G'
 

    % UPDATE STEP

    if (rand(1) <= Prob_sensor)
        S_cov_res = H*P_pred(:,:,t+1)*H' + R_cov_pos;
        W_Kal_gain = P_pred(:,:,t+1)*H'*inv(S_cov_res);
        pos_est(:,t+1) = pos_pred(:,t+1) + W_Kal_gain*(pos_meas(:,t+1) - H*pos_pred(:,t+1));
        P_store(:,:,t+1) = (eye(3) - W_Kal_gain*H)*P_pred(:,:,t+1);
    else 
        pos_est(:,t+1) = pos_pred(:,t+1);
        P_store(:,:,t+1) = P_pred(:,:,t+1);
    end
    
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
                sum_delta_z = sum_delta_z + q_ij * (meas_distance(3) - ideal_distance(3)); 
            end
        end
        
        % Ideal velocity without repulsion from obstacles
        distance = sqrt(sum_delta_x^2 + sum_delta_y^2 + sum_delta_z^2); 
        yaw = atan2(sum_delta_y, sum_delta_x); % Calculate angle in the xy-plane
        pitch = acos(sum_delta_z/sqrt(sum_delta_x^2 + sum_delta_y^2 + sum_delta_z^2) ); % Calculate angle with the z-axis 
        if distance > d_max
            distance = d_max;
        end
        % Update velocities in x, y, and z directions
        vel_store(i, 1, t+1) = vel_store(1, 1, t) + KF * distance * cos(yaw) * sin(pitch); 
        vel_store(i, 2, t+1) = vel_store(1, 2, t) + KF * distance * sin(yaw) * sin(pitch); 
        vel_store(i, 3, t+1) = vel_store(1, 3, t) + KF * distance * cos(pitch);

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
        % Optimize seabed repulsion objects only in the location of the agents
        ob_seabed_short=ob_seabed((int32(max(floor(min(min(pos_store(:,2,t))))*0.01*numel(sampled_k)-1,1))):(int32(min(floor(max(max(pos_store(:,2,t))))*0.01*numel(sampled_k)+1,numel(sampled_k)))),:,(int32(max(floor(min(min(pos_store(:,1,t))))*0.01*numel(sampled_k)-1,1))):(int32(min(floor(max(max(pos_store(:,1,t))))*0.01*numel(sampled_k)+1,numel(sampled_k)))));
        ob_pose = [ob_temp; obs_pose; reshape(permute(ob_seabed_short, [1 3 2]), [], 3)];
        repulsion = computeRepulsion3D([pos_store(i,1,t), pos_store(i,2,t), pos_store(i,3,t)], ob_pose(1:length(ob_temp)+length(obs_pose),:), detect_R);
        repulsion = repulsion + computeRepulsion3D([pos_store(i,1,t), pos_store(i,2,t), pos_store(i,3,t)], ob_pose(length(ob_temp)+length(obs_pose):end,:), detect_R)*2;
        % New follower velocity
        vel_store(i,:,t+1) = vel_store(i,:,t+1) + beta*repulsion;
        
        % Update followers position and velocity
        out = speedLimit3D(vel_store(i, :, t), vel_store(i, :, t+1), dt, max_vel, max_acc);
        vel_store(i, 1, t+1) = out(1);
        vel_store(i, 2, t+1) = out(2);
        vel_store(i, 3, t+1) = out(3); 
        pos_store(i, 1, t+1) = pos_store(i, 1, t) + dt * vel_store(i, 1, t+1); % x
        pos_store(i, 2, t+1) = pos_store(i, 2, t) + dt * vel_store(i, 2, t+1); % y
        pos_store(i, 3, t+1) = pos_store(i, 3, t) + dt * vel_store(i, 3, t+1); % z
        ang_store(i,1,t+1) = atan2(norm([vel_store(i,2,t+1), vel_store(i,3,t+1)]), vel_store(i,1,t+1)); % yaw
        ang_store(i,2,t+1) = acos(vel_store(i,3,t+1)/sqrt( vel_store(i,1,t+1)^2+ vel_store(i,2,t+1)^2+ vel_store(i,3,t+1)^2)); % pitch
    end
    
    %% Reaching goal

    now = [pos_store(1,1,t+1),pos_store(1,2,t+1),pos_store(1,3,t+1)];

    flag_leader_close_to_anchor(2) = flag_leader_close_to_anchor(1); % store old value
    if norm(now-goal)<ext_range
        flag_leader_close_to_anchor(1) = 1;
        if flag_leader_close_to_anchor(2) == 0
            disp('Leader has seen the anchor');
        end
    else
        flag_leader_close_to_anchor(1) = 0;
    end

    % Distance followers-anchor
    for i=1:n
        sub_pos = [pos_store(i,1,t+1),pos_store(i,2,t+1),pos_store(i,3,t+1)];
        if i == 1
            min_anchor_distance = norm(sub_pos-goal);
            fol_detect_num = 1;
        else
            if norm(sub_pos-goal) < min_anchor_distance
                min_anchor_distance = norm(sub_pos-goal);
                fol_detect_num = i;
            end
        end
    end
    
    ext_meas(2) = ext_meas(1);
    if min_anchor_distance<ext_range
        ext_meas(1) = 1; % Exteroceptive sensors activated (submarines can see the anchor)
        if ext_meas(2) == 0
            if not(fol_detect_num == 1)
                string = ['Follower number ', num2str(fol_detect_num-1), ' has seen the anchor'];
                disp(string);
            end
        end
    else
        ext_meas(1) = 0; % Exteroceptive sensors deactivated
    end

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
                if abs(vel_store(i,1,t))<=sigma_fol_dist && abs(vel_store(i,2,t))<=sigma_fol_dist && abs(vel_store(i,3,t))<=sigma_fol_dist
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
            % Eliminate drift error by resetting proprioceptive measurements
            % while leaving the anchor
            x_meas_prop(2) = pos_est(1,t); % starting x value for the next integration (until next anchor is reached)
            y_meas_prop(2) = pos_est(2,t); % starting y value for the next integration (until next anchor is reached)
            z_meas_prop(2) = pos_est(3,t); % starting z value for the next integration (until next anchor is reached)
            v_meas(1,t+1) = (pos_est(1,t)-pos_est(1,t-1))/dt; % starting vx value for the next integration (until next anchor is reached)
            v_meas(2,t+1) = (pos_est(2,t)-pos_est(2,t-1))/dt; % starting vy value for the next integration (until next anchor is reached)
            v_meas(3,t+1) = (pos_est(3,t)-pos_est(3,t-1))/dt; % starting vz value for the next integration (until next anchor is reached)
        end
    end

    %% ==== Animation ====
    arrow_length=0.8;
    figure(1), clf, hold on;
    colormap(gray);
    
    plot3(squeeze(pos_store(1,1,2:t+1)),squeeze(pos_store(1,2,2:t+1)),squeeze(pos_store(1,3,2:t+1)),'b','LineWidth',0.5); % plot path
    plot3(squeeze(pos_est(1,2:t+1)),squeeze(pos_est(2,2:t+1)),squeeze(pos_est(3,2:t+1)),'r','LineWidth',0.5); % plot path
    plot3(squeeze(pos_meas(1,2:t+1)),squeeze(pos_meas(2,2:t+1)),squeeze(pos_meas(3,2:t+1)),'g','LineWidth',0.5); % plot path

    % plot's view properties
    view(mod(t/2,365),45); % azimuth angle, elevation angle
    % Define view properties
    x_min = max(min(min(pos_store(:,1,t))) - 20, 0);
    x_max = min(max(max(pos_store(:,1,t))) + 20, 100);
    y_min = max(min(min(pos_store(:,2,t))) - 20, 0);
    y_max = min(max(max(pos_store(:,2,t))) + 20, 100);
    z_min = -57;
    z_max = max(max(pos_store(:,3,t))) + 10;
    % Use set instead of axis due to computation time checked with profile(Profile execution time for functions) 
%     axis([xmin xmax ymin ymax zmin zmax]);
    
    % Apply view properties using set
    set(gca, 'XLim', [x_min x_max], 'YLim', [y_min y_max], 'ZLim', [z_min z_max]);
    
    % Use drawArrow3D instead of quiver3 due to computation time checked with profile(Profile execution time for functions) 
%     quiver3(pos_store(:,1,t+1),pos_store(:,2,t+1),pos_store(:,3,t+1),arrow_length*vel_store(:,1,t+1),arrow_length*vel_store(:,2,t+1),arrow_length*vel_store(:,3,t+1),'k');
    drawArrow3D([pos_store(:,1,t+1),pos_store(:,2,t+1),pos_store(:,3,t+1)],[pos_store(:,1,t+1)+arrow_length*vel_store(:,1,t+1),pos_store(:,2,t+1)+arrow_length*vel_store(:,2,t+1),pos_store(:,3,t+1)+arrow_length*vel_store(:,3,t+1)],[1, 0.640, 0],2);

    % draw obstacles
    if size(ob_temp)~=[0 0]
            plot3(ob_temp(:,1),ob_temp(:,2),ob_temp(:,3),'r.','MarkerSize',20*detect_R/2);
            hold on;
    end
%     Draw dynamic seabed profile Obstacles
%     ob=reshape(permute(ob_seabed_short, [1 3 2]), [], 3);
%     if size(ob)~=[0 0]
%             plot3(ob(:,1),ob(:,2),ob(:,3),'r.','MarkerSize',20);
%             hold on;
%     end
    
    for j=1:n
        if j==1    % drawing leader in a different colour
            plotPyramid3d(pos_store(j,1,t+1), pos_store(j,2,t+1), pos_store(j,3,t+1), ang_store(j,1,t+1), ang_store(j,2,t+1),'r');
        else
            plotPyramid3d(pos_store(j,1,t+1), pos_store(j,2,t+1), pos_store(j,3,t+1), ang_store(j,1,t+1), ang_store(j,2,t+1),'g');
        end        
    end

    % draw the communication direction between two agents
    for i=1:n
        for j=1:n
            if A(i,j)==1
                drawArrow3D([pos_store(j,1,t+1),pos_store(j,2,t+1),pos_store(j,3,t+1)],[pos_store(i,1,t+1),pos_store(i,2,t+1),pos_store(i,3,t+1)], [0.678, 0.847, 0.902],0.5);
            end
        end
    end

    drawMap(myMap);

    plot3(goal(1),goal(2),goal(3),'Xy','LineWidth',3);
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
%     legend('Real', 'Estimated', 'Measured');
    drawnow;
end

% display fig1's legend at the and due to computation time checked with profile(Profile execution time for functions) 
figure(1), hold on;
legend('Real', 'Estimated', 'Measured');
hold off;

%% Draw path

color = [
    1, 0, 1;     % Magenta
    0, 0.5, 0;   % Green
    0, 0, 1;     % Blue
    0, 0, 0;     % Black
    1, 0, 0;     % Red
    0, 1, 1;     % Cyan
    1, 1, 0;     % Yellow
    1, 0.647, 0; % Orange
];
type=[2,1,0.5,0.5,2,2];     % different line type

figure(2), clf, hold on;    % Draw the path record of formation 
for i=1:n
    plot3(squeeze(pos_store(i,1,2:t)),squeeze(pos_store(i,2,2:t)),squeeze(pos_store(i,3,2:t)),'Color', color(i,:),'LineWidth',2); % plot path
end

% ob_pose=[ob_temp; obs_pose; reshape(permute(ob_seabed, [1 3 2]), [], 3)];
ob_seabed=reshape(permute(ob_seabed, [1 3 2]), [], 3);
if size(ob_seabed)~=[0 0]
    plot3(ob_seabed(:,1),ob_seabed(:,2),ob_seabed(:,3),'k.','LineWidth',10); % plot obstacles
end
if size(ob_temp)~=[0 0]
    plot3(ob_temp(:,1),ob_temp(:,2),ob_temp(:,3),'r.','MarkerSize',20*detect_R/2); % plot obstacles
end
if size(myMap.anchors_x)~=[0 0]
    plot3(myMap.anchors_x(1,2:end),myMap.anchors_y(1,2:end),myMap.anchors_z(1,2:end)+15,'.','Color',[0.51, 0, 0.51],'MarkerSize',20); % plot goals
end

for i=2:n
    plot3(squeeze(pos_store(i,1,2)),squeeze(pos_store(i,2,2)),squeeze(pos_store(i,3,2)),'bp','Color', color(i,:),'LineWidth',1);    % plot followers' starting positions
    plot3(squeeze(pos_store(i,1,t)),squeeze(pos_store(i,2,t)),squeeze(pos_store(i,3,t)),'m^','Color', color(i,:),'LineWidth',2);    % plot followers' final positions
end
plot3(squeeze(pos_store(1,1,2)),squeeze(pos_store(1,2,2)),squeeze(pos_store(1,3,2)),'*','Color', color(1,:),'LineWidth',1); % plot leader's starting point
plot3(squeeze(pos_store(1,1,t)),squeeze(pos_store(1,2,t)),squeeze(pos_store(1,3,t)),'o','Color', color(1,:),'LineWidth',2); % plot leader's final point

grid on;
legend('Leader','Follower 1','Follower 2','Follower 3','Follower 4','Follower 5','Follower 6','Follower 7','Seabed Obs','Obstacles', 'Goals');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)')
title('Trajectories');
view(3); % Ensure the plot is shown in 3D view

%% Plot KF results

figure(3), clf;

% Subplot for x values
subplot(3,1,1);
hold on;
plot(squeeze(pos_store(1,1,3:t)), 'b');
plot(pos_est(1,3:t), 'r');
plot(pos_meas(1,3:t), 'g');
xlabel('t [s]');
ylabel('x [m]');
legend('Real', 'Estimated', 'Measured');
title("Leader's x values");
hold off;

% Subplot for y values
subplot(3,1,2);
hold on;
plot(squeeze(pos_store(1,2,3:t)), 'b');
plot(pos_est(2,3:t), 'r');
plot(pos_meas(2,3:t), 'g');
xlabel('t [s]');
ylabel('y [m]');
legend('Real', 'Estimated', 'Measured');
title("Leader's y values");
hold off;

% Subplot for z values
subplot(3,1,3);
hold on;
plot(squeeze(pos_store(1,3,3:t)), 'b');
plot(pos_est(3,3:t), 'r');
plot(pos_meas(3,3:t), 'g');
xlabel('t [s]');
ylabel('z [m]');
legend('Real', 'Estimated', 'Measured');
title("Leader's z values");
hold off;