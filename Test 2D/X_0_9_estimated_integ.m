clc;
clear;
%addpath('C:\Users\Arianna\Desktop\DistributedEstimation\Test 2D')

% Time
dt = 0.18;
time = 0:dt:150;

%% Parameters
d_max = 2;          % above d_max, velocity remains constant 
detect_R = 4;       % distance to detect obstacles
ob_num = 10;        % obstacles number
beta = 14;          % object repulsion parameter
m = 1;              % local minima detection
KL = 1;             % leader's velocity coeff. KL<KF in order to "speed up" followers' movements
KF = 1.2;           % followers' velocity coeff. Be aware to increase more than 1, velocity & obstacles' repulsion are correlated
max_vel = 6;        % maximum velocity
max_acc = 3;        % maximum acceleration
fol_num = 5;        % number of followers
n = fol_num+1;      % fol_num followers and 1 leader

%% Model

A_model = eye(2); % x, y
B_model = eye(2)*dt; % vx, vy

%% Define the structure of MAP

myMap = struct();

myMap.dim=100;                                      % Map dimension
myMap.k = 0:0.1:myMap.dim;                          % Time values from 0 to max dimension with a step of 0.1
myMap.profile = zeros(size(myMap.k));               % Initialize the hill profile
myMap.num_sin=30;                                   % number of sinusoids
myMap.amp=rand(1, myMap.num_sin)*2;                 % Random amplitudes between 0 and 2
myMap.phase=rand(1, myMap.num_sin)* 2*pi;           % Random phase shifts between 0 and 2*pi
myMap.freq=linspace(0.005, 0.1, myMap.num_sin)*0.6; % Frequencies of the sinusoids
myMap.ground_zero = 40;   % zero level of the sea

myMap.anchors_x = [5, 20, 40, 65, 80, 95]; % Declare the anchor array
myMap.anchors_y = zeros(size(myMap.anchors_x));
myMap.anchor_offset=15;

% Generate the seabed profile by summing sinusoids
for i = 1:myMap.num_sin
    myMap.profile = myMap.profile + myMap.amp(i) * sin(2*pi*myMap.freq(i)*myMap.k + myMap.phase(i));
end
myMap.profile = myMap.profile - myMap.ground_zero; %set level zero
% anchors_y
for i = 1:length(myMap.anchors_x)
    myMap.anchors_y(i) = interp1(myMap.k, myMap.profile, myMap.anchors_x(i));
end
  
% Obstacles
ob_temp=obstBuild(myMap, ob_num, detect_R);

% Create ob_seabed as avoidance objects
ob_seabed = [myMap.k(detect_R:detect_R:end); myMap.profile(detect_R:detect_R:end)]';% arr(x:y:end) start from x, take every y elements until end of vector arr

% Concatenate ob_temp with the seabed
ob_pose = [ob_temp; ob_seabed];

% First goal
goal = [myMap.anchors_x(1), 0];
anchor_num = 1;

fin_goal_reached = 0;

%% Initial state

pos_store = zeros(n, 2, length(time)); % actual states [x, y]
theta_store = zeros(n, length(time)); % actual orientation [theta]
vel_store = zeros(n, 2, length(time)); % actual inputs [vx, vy]

% Initial position 
p_0 = zeros(n,2); % [x, y] for n submarines
p_0(1,1:2) = [5, 0]; %leader's first position known
for i=2:n
    p_0(i,1) = 4*rand(1);
    p_0(i,2) = -2-detect_R/2*i*rand(1);
end

% Initial velocity
v_0 = zeros(n,2); % [vx, vy] for for n submarines

% Define triangles initial state 
pos_store(:,:,2:3) = cat(3, p_0, p_0); % starting from time=2
vel_store(:,:,2:3) = cat(3, v_0, v_0); % starting from time=2

%% Uncertainty

% Velocity (input) measurements
mu_vel = 0;
sigma_vel = 0.2;

% Process noise covariance matrix
% Q_cov = rand(2,2)*sigma_vel; 
% Q_cov = Q_cov'*Q_cov;
% disp(Q_cov)
Q_cov = [0.026, 0.031; 0.031, 0.038]; % error on system input to account for other disturbances

% Process noise gain matrix
G = eye(2);

% Position (state) measurements

% Proprioceptive sensor
mu_acc = 0.05; 
sigma_acc = 0.1;
H = eye(2);
Prob_sensor = 0.25; % new measurements provided every 4 steps (on average)

% Proprioceptive measurement noise covariance matrix
% R_cov = rand(2,2)*sigma_acc;
% R_cov = R_cov'*R_cov;
% disp(R_cov)
R_cov_acc = [0.005, 0.002; 0.002, 0.004];

% Exteroceptive sensors
mu_pos = 0; 
sigma_pos = 0.1; 
accuracy_ratio = 3; % how much ext meas are considered to be more reliable than proprioceptive meas

% Exteroceptive measurement noise covariance matrix
% R_cov = rand(2,2)*sigma_pos;
% R_cov = R_cov'*R_cov;
% disp(R_cov)
R_cov_pos = [0.28, 0.25; 0.25, 0.26];

% leader is initially on the first anchor: exteroceptive meas activated
ext_range = 5; % if distance between a submarine and the anchor is less than ext_range, ext meas are activated
ext_meas = [1, 1]; % [value at current time instant, value at previous time instant] (ext meas initially activated)
flag_leader_close_to_anchor = [1,1]; % [value at current time instant, value at previous time instant] (leader starts on anchor)

% Predicted, estimated and measured position (Kalman filter)
pos_pred = zeros(2, length(time)); % predicted position (x,y)
pos_est = zeros(2, length(time)); % estimated position (x,y)
pos_meas = zeros(2, length(time)); % position measurements (x,y)
v_meas = zeros(2, length(time)); % velocity measurements (x,y)
acc_meas = zeros(2, length(time)); % acceleration measurements (x,y)

% First state measurement and estimated state initialization
pos_meas(:,2:3) = pos_store(1,:,2:3) + mvnrnd(mu_pos, sigma_pos)';
pos_est(:,2:3) = pos_meas(:,2:3);
x_init = pos_meas(1,3);
y_init = pos_meas(2,3);
v_meas(1,2:3) = 0;
v_meas(2,2:3) = 0;
x_meas_prop = [x_init, x_init]; % [new_meas, old_meas] initialization
y_meas_prop = [y_init, y_init]; % [new_meas, old_meas] initialization

% Error covariance matrix of state estimate
P_init = [2, 2; 2, 2]; % Symmetric and positive semidefinite
P_store = zeros(2, 2, length(time));
P_store(:,:,2:3) = cat(3, P_init, P_init); % Initialization
P_pred = zeros(2, 2, length(time));

%% Consensus

A=[0 1 1 1 1 1;     % a(ij)
   1 0 0 0 1 1;
   1 0 0 1 0 1;
   1 0 1 0 1 0;
   1 1 0 1 0 0;
   1 1 1 0 0 0];

% Consensus relative position 
delta_x=[0 1.5 -3 -1.5 1.5 -1.5]*2.5;  
delta_y=[0 1.5 0 -1.5 -1.5 1.5]*2.5;  % relative position between leader and followers [m]

% Uncertainty on followers relative distance measurements
mu_fol_dist = 0;
sigma_fol_dist = 1; % [m]

% Followers measurement noise covariance matrix
% R_cov = rand(2,2)*sigma_fol_dist;
% R_cov = R_cov'*R_cov;
% disp(R_cov)
R_cov_fol = [0.42, 0.50; 0.50, 0.99];

%% Run

for t=3:length(time)-2

    % Calculate attraction from anchor and compute leader velocity 
    distance = norm(pos_est(:,t)-goal); % pos_est(t) because leader doesn't know real position (pos_store(t))
    th=atan2(goal(2)-pos_est(2,t),goal(1)-pos_est(1,t));
    if distance>d_max
        distance=d_max;
    end
    vx = KL*distance*cos(th);
    vy = KL*distance*sin(th);

    %% Calculate new leader velocity with obstacles

    repulsion=computeRepulsion([pos_store(1,1,t),pos_store(1,2,t)],ob_pose(1:(length(ob_seabed)+length(ob_temp)),:),detect_R);  

    % New leader velocity
    vx = vx + beta/3*repulsion(1);
    vy = vy + beta/3*repulsion(2);

    % When the local minimum appears, add a random error
    if(distance>1 && abs(vx)<=0.08 && abs(vy)<=0.08)
        out(1) = -min(15,m/dt);  % go back along x direction (vx=-10->x=-1 if dt=0.1)
        out(2) = m*dt*(2*m+rand(1)); % move +y randomly
        m = m+1;
        disp("Trying to avoid a challenging obstacle...");
    else
        out = speedLimit_V2([vel_store(1,1,t) vel_store(1,2,t)],[vx vy],dt,max_vel,max_acc);
    end

    % Update pos_store and vel_store (real values) for leader
    vel_store(1,1,t+1) = out(1);                                        % vx
    vel_store(1,2,t+1) = out(2);                                        % vy
    pos_store(1,1,t+1) = pos_store(1,1,t) + dt*vel_store(1,1,t+1);      % x
    pos_store(1,2,t+1) = pos_store(1,2,t) + dt*vel_store(1,2,t+1);      % y
    theta_store(1,t+1) = atan2(vel_store(1,2,t+1), vel_store(1,1,t+1)); % theta

    %% Measurements
    
    % MEASUREMENTS

    % New acc measurement (proprioceptive sensor)
    real_acc = [(vel_store(1,1,t+1)-vel_store(1,1,t))/dt, (vel_store(1,2,t+1)-vel_store(1,2,t))/dt]; % real_acc
    acc_meas(:,t+1) = real_acc + mvnrnd(mu_acc, sigma_acc)'; % real acc + measurement error
    % First integration
    v_meas(1,t+1) = runge_kutta_4(acc_meas(1,t), acc_meas(1,t+1), dt, v_meas(1,t));
    v_meas(2,t+1) = runge_kutta_4(acc_meas(2,t), acc_meas(2,t+1), dt, v_meas(2,t));
    % Second integration
    x_meas_prop(1) = runge_kutta_4(v_meas(1,t), v_meas(1,t+1), dt, x_meas_prop(2));
    y_meas_prop(1) = runge_kutta_4(v_meas(2,t), v_meas(2,t+1), dt, y_meas_prop(2));

    % real_acc = [(vel_store(1,1,t+1)-vel_store(1,1,t))/dt, (vel_store(1,2,t+1)-vel_store(1,2,t))/dt]; % real acc
    % acc_meas(:,t+1) = real_acc + mvnrnd(mu_acc, sigma_acc)'; % real acc + measurement error
    % delta_pos_prop = v0*dt + 0.5*acc_meas(:,t+1)*dt^2; % delta = v0*dt + 0.5*acc_meas*dt^2
    % v0 = v0 + acc_meas(:,t+1)*dt; % new initial velocity (for the next time instant)
    
    % Store results
    x_meas = x_meas_prop(1);
    y_meas = y_meas_prop(1);

    % When a follower is close to the anchor, exteroceptive measurements are activated.
    if (ext_meas(1) == 1 || ext_meas(2) == 1) && (flag_leader_close_to_anchor(1) == 0 && flag_leader_close_to_anchor(2) == 0) % only followers are close to the anchor
        ext_meas = pos_store(1,:,t+1) + mvnrnd(mu_pos, sigma_pos)';
        x_meas = ext_meas(1);
        y_meas = ext_meas(2);
    end
        
    % If the leader is close to an anchor, it can measure more
    % accurately its position. In this case it completely relies on exteroceptive measurements.
    if flag_leader_close_to_anchor(1) == 1 || flag_leader_close_to_anchor(2) == 1 % leader can see the anchor
        ext_meas = pos_store(1,:,t+1) + mvnrnd(mu_pos, sigma_pos)';
        x_meas = ext_meas(1);
        y_meas = ext_meas(2);
    end

    % Store measurements to be considered by the KF
    pos_meas(:,t+1) = [x_meas; y_meas];

    % Store initial values for the next integration step (prop meas)
    x_meas_prop(2) = x_meas_prop(1);
    y_meas_prop(2) = y_meas_prop(1);

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
        P_store(:,:,t+1) = (eye(2) - W_Kal_gain*H)*P_pred(:,:,t+1);
    else 
        pos_est(:,t+1) = pos_pred(:,t+1);
        P_store(:,:,t+1) = P_pred(:,:,t+1);
    end
    
    %% Calculate followers' velocity

    for i=2:n  % starts from 2, leader is out      
        sum_delta_x = 0;
        sum_delta_y = 0;
        for j=1:n       % Linear Consensus control
                        % Condition Check: if A(i,j)==1: Checks if there is an edge between nodes i and j in the network represented by the adjacency matrix A.
                        % Edge Weight Calculation: Computes the edge weight q_ij based on Metroplis-Hastings weights
                        % Update Summations: Updates the sum of differences in x and y coordinates weighted by q_ij.
            if A(i,j)==1
                q_ij = 1/(max(sum(A(i,:)), sum(A(j,:))) + 1); 
                ideal_distance = [delta_x(j)-delta_x(i), delta_y(j)-delta_y(i)];
                actual_distance = pos_store(j,:,t)-pos_store(i,:,t);
                meas_distance = actual_distance + mvnrnd(mu_fol_dist, sigma_fol_dist)';
                % Accumulate contributes
                sum_delta_x = sum_delta_x + q_ij*(meas_distance(1)-ideal_distance(1));                   
                sum_delta_y = sum_delta_y + q_ij*(meas_distance(2)-ideal_distance(2));
            end
        end

        % Ideal velocity without repulsion from obstacle
        distance = sqrt(sum_delta_x^2 + sum_delta_y^2);
        th = atan2(sum_delta_y, sum_delta_x);
        if distance>d_max
            distance = d_max;
        end
        vel_store(i,1,t+1) = vel_store(1,1,t) + KF*distance*cos(th);    
        vel_store(i,2,t+1) = vel_store(1,2,t) + KF*distance*sin(th);

       % Consider repulsion among submarines and store their pose to obs_pose
        kk = 0;
        obs_pose = zeros(fol_num,2);
        for j=1:n
            if j~=i
                kk = kk+1;
                obs_pose(kk,1) = pos_store(j,1,t);
                obs_pose(kk,2) = pos_store(j,2,t);
            end
        end
        ob_pose = [ob_temp; ob_seabed; obs_pose];
        repulsion = computeRepulsion([pos_store(i,1,t),pos_store(i,2,t)],ob_pose,detect_R);        
        % New follower velocity
        vel_store(i,:,t+1) = vel_store(i,:,t+1) + beta*repulsion;

        % Update followers position and velocity
        out=speedLimit_V2([vel_store(i,1,t) vel_store(i,2,t)],[vel_store(i,1,t+1) vel_store(i,2,t+1)],dt,max_vel,max_acc);
        vel_store(i,1,t+1) = out(1);
        vel_store(i,2,t+1) = out(2);
        pos_store(i,1,t+1) = pos_store(i,1,t) + dt*vel_store(i,1,t+1);
        pos_store(i,2,t+1) = pos_store(i,2,t) + dt*vel_store(i,2,t+1);
        theta_store(i,t+1) = atan2(vel_store(i,2,t+1),vel_store(i,1,t+1));
    end

    %% Reaching goal
    
    now = [pos_store(1,1,t+1),pos_store(1,2,t+1)];

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
        sub_pos = [pos_store(i,1,t+1),pos_store(i,2,t+1)];
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
                if abs(vel_store(i,1,t))<=0.5*sigma_fol_dist && abs(vel_store(i,2,t))<=0.5*sigma_fol_dist
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
            goal = [myMap.anchors_x(anchor_num) (myMap.anchors_y(anchor_num)+myMap.anchor_offset)];
        
            % Eliminate drift error by resetting proprioceptive measurements
            % while leaving the anchor
            x_meas_prop(2) = pos_est(1,t); % starting x value for the next integration (until next anchor is reached)
            y_meas_prop(2) = pos_est(2,t); % starting y value for the next integration (until next anchor is reached)
            v_meas(1,t+1) = (pos_est(1,t)-pos_est(1,t-1))/dt; % starting vx value for the next integration (until next anchor is reached)
            v_meas(2,t+1) = (pos_est(2,t)-pos_est(2,t-1))/dt; % starting vy value for the next integration (until next anchor is reached)
         end
    end    

    %% ==== Animation ====
    hold off;
    ArrowLength=0.5;
    plot(squeeze(pos_store(1,1,2:t)),squeeze(pos_store(1,2,2:t)),'b','LineWidth',0.5); % plot path
    hold on;
    plot(squeeze(pos_est(1,2:t)),squeeze(pos_est(2,2:t)),'r','LineWidth',0.5); % plot path
    plot(squeeze(pos_meas(1,2:t)),squeeze(pos_meas(2,2:t)),'g','LineWidth',0.5); % plot path

    for j=1:n
        quiver(pos_store(j,1,t+1),pos_store(j,2,t+1),ArrowLength*cos(theta_store(j,t+1)),ArrowLength*sin(theta_store(j,t+1)),'k');
        hold on;
        if j==1    % drawing leader in a different colour
            plotTriangle2d_V2(pos_store(j,1,t+1), pos_store(j,2,t+1), theta_store(j,t+1),'r');
        else
            plotTriangle2d_V2(pos_store(j,1,t+1), pos_store(j,2,t+1), theta_store(j,t+1),'g');
        end        
    end
    % draw the communication direction between two agents
    for i=1:n
        for j=1:n
            if A(i,j)==1
                drawArrow([pos_store(j,1,t+1),pos_store(j,2,t+1)],[pos_store(i,1,t+1),pos_store(i,2,t+1)], .5);
                hold on;
            end
        end
    end
    % draw obstacles
    if size(ob_temp)~=[0 0]
        plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);
        hold on;
    end
    seabedInitialization_2d_V2(myMap);
    plot(goal(1),goal(2),'Xr','LineWidth',2)
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    legend('Real', 'Estimated', 'Measured');
    hold on;
    drawnow;    
end

%% Draw path

color='mgbkrc';             % corresponding to 6 colors
type=[2,1,0.5,0.5,2,2];     % different line type

figure(1), clf, hold on;    % Draw the path record of formation 
for i=1:n
    plot(squeeze(pos_store(i,1,2:t)),squeeze(pos_store(i,2,2:t)),color(1,i),'LineWidth',2); % plot path
end
for i=2:n
    plot(squeeze(pos_store(i,1,2)),squeeze(pos_store(i,2,2)),'bp','color',color(1,i),'LineWidth',1);    % plot followers' starting positions
    plot(squeeze(pos_store(i,1,t)),squeeze(pos_store(i,2,t)),'m^','color',color(1,i),'LineWidth',2);    % plot followers' final positions
end
plot(squeeze(pos_store(1,1,2)),squeeze(pos_store(1,2,2)),'*','color',color(1,1),'LineWidth',1); % plot leader's starting point
plot(squeeze(pos_store(1,1,t)),squeeze(pos_store(1,2,t)),'o','color',color(1,1),'LineWidth',2); % plot leader's final point

if size(ob_pose)~=[0 0]
    plot(ob_pose(:,1),ob_pose(:,2),'Xk','LineWidth',2); % plot obstacles
end
grid on;
xlabel('x');
ylabel('y');
legend('Leader','Follower 1','Follower 2','Follower 3','Follower 4','Follower5');
xlabel('x(m)');
ylabel('y(m)');
title('Trajectories');

%% Plot KF results

figure(2), clf, hold on;
plot(squeeze(pos_store(1,1,3:t)), 'b');
plot(pos_est(1,3:t), 'r');
plot(pos_meas(1,3:t), 'g');
xlabel('t [s]');
ylabel('x [m]');
legend('Real', 'Estimated', 'Measured');
title("Leader's x values");

figure(3), clf, hold on;
plot(squeeze(pos_store(1,2,3:t)), 'b');
plot(pos_est(2,3:t), 'r');
plot(pos_meas(2,3:t), 'g');
xlabel('t [s]');
ylabel('y [m]');
legend('Real', 'Estimated', 'Measured');
title("Leader's y values");

