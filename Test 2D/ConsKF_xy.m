clc;
clear;

% Time
dt = 0.18;
time = 0:dt:150;

%% Parameters
d_max = 2;          % above d_max, velocity remains const
detect_R = 4;       % distance to detect obstacles (affects max velocity in speedLimit)
ob_num = 10;        % obstacles number
beta = 15;          % object repulsion parameter
m = 1;              % local minima detection
KN = 0.8;           % leader's velocity coeff. Kn<K0 in order to "speed up" followers' movements
K0 = 1;             % followers' velocity coeff. Be aware to increase more than 1, velocity & obstacles' repulsion are correlated
fol_num = 5;        % number of followers
n = fol_num+1;        % fol_num followers and 1 leader
gamma = 1;          % t+1 followers velocity multiplier (useful for maintaining relative position)             

%% Uncertainty
Prob_sensor = 0.3;
p_store = zeros(n, 3, length(time)); % states [x, y, theta]
v_store = zeros(n, 2, length(time)); % velocities [vx, vy]
% Velocity errors
mu_vel = 0;
sigma_vel = 0.5;
% Position measurements
mu_pos = 0.1; % proprioceptive sensors
sigma_pos = 3;
mu_pos_ext = 0;
sigma_pos_ext = 0.1; % exteroceptive sensors

% Kalman filter
pos_est = zeros(2, length(time)); % estimated position (x,y)
pos_meas = zeros(2, length(time)); % position measurements (x,y)
x_P1 = 5;
y_P1 = 5;
P1Store = zeros(2, length(time));
P1PredStore = zeros(2, length(time));

%% Consensus

A=[0 1 0 1 0 1;     % a(ij)
   1 0 0 0 1 1;
   0 0 0 1 0 1;
   1 0 1 0 1 0;
   0 1 0 1 0 0;
   1 1 1 0 0 0];
% consensus relative position 
delta_x=[0 1.5 -3 -1.5 1.5 -1.5]*2.5;  
delta_y=[0 1.5 0 -1.5 -1.5 1.5]*2.5;  % relative position between leader and follwers

%% Initial state

% Initial position matrix
p_0=zeros(n,3); % [x, y, theta] for fol_num followers + leader
v_0=zeros(n,2); % [vx, vy] for fol_num followers + leader
for i=1:n
    p_0(i,1)=4*rand(1);
    p_0(i,2)=25+detect_R/2*i*rand(1);
end

% Define triangles initial state 
p_store(:,:,2) = p_0; % starting from time=2 
v_store(:,:,2) = v_0; % starting from time=2

pos_meas(1,2) = p_store(1,1,2) + randn(1)*sigma_pos + mu_pos; % first x measurement
pos_est(1,2) = pos_meas(1,2);
pos_meas(2,2) = p_store(1,2,2) + randn(1)*sigma_pos + mu_pos; % first y measurement
pos_est(2,2) = pos_meas(2,2);

vx_bar = 0 + randn(1)*sigma_vel + mu_vel;
vy_bar = 0 + randn(1)*sigma_vel + mu_vel;

%% Define the structure of MAP
myMap = struct();

myMap.dim=100;                                      % Map dimension
myMap.k = 0:0.1:myMap.dim;                          % Time values from 0 to max dimension with a step of 0.1
myMap.profile = zeros(size(myMap.k));               % Initialize the hill profile
myMap.num_sin=30;                                   % number of sinusoids
myMap.amp=rand(1, myMap.num_sin)*2;                 % Random amplitudes between 0 and 2
myMap.phase=rand(1, myMap.num_sin)* 2*pi;           % Random phase shifts between 0 and 2*pi
myMap.freq=linspace(0.005, 0.1, myMap.num_sin)*0.6; % Frequencies of the sinusoids

myMap.anchors_x = [20, 40, 65, 80, 95]; % Declare the anchor array
myMap.anchors_y = zeros(size(myMap.anchors_x));
myMap.anchor_offset=15;

% Generate the seabed profile by summing sinusoids
for i = 1:myMap.num_sin
    myMap.profile = myMap.profile + myMap.amp(i) * sin(2*pi*myMap.freq(i)*myMap.k + myMap.phase(i));
end
% anchors_y
for i = 1:length(myMap.anchors_x)
    myMap.anchors_y(i) = interp1(myMap.k, myMap.profile, myMap.anchors_x(i));
end
  
% Obstacles
ob_temp=obstBuild(myMap, ob_num, detect_R);

% First goal
goal = [myMap.anchors_x(1), (myMap.anchors_y(1)+myMap.anchor_offset)];
anchor_num = 1;

fin_goal_reached = 0;

%% Run
for t=2:length(time)-1
    % Calculate attraction from anchor and put it on leader velocity (first triangle)
    distance=sqrt((goal(1)-pos_est(1,t))^2+(goal(2)-pos_est(2,t))^2); % pos_est(t) because leader doesn't know real position (p_store(t))
    th=atan2(goal(2)-pos_est(2,t),goal(1)-pos_est(1,t));
    if distance>d_max
        distance=d_max;
    end
    vx = KN*distance*cos(th);
    vy = KN*distance*sin(th);

    %% Calculate leader's new velocity with obstacles

    % create ob_seabed as avoidance objects
    ob_seabed = [myMap.k(detect_R:detect_R:end); myMap.profile(detect_R:detect_R:end)]';% arr(x:y:end) start from x, take every y elements until end of vector arr
    % Concatenate ob_temp with the seabed
    ob_pose = [ob_temp; ob_seabed];
    repulsion=computeRepulsion([p_store(1,1,t),p_store(1,2,t)],ob_pose,detect_R);        
    % Put obstacle repulsion to leader
    vx = vx + beta/3.5*repulsion(1);
    vy = vy + beta/3.5*repulsion(2);

    % When the local minimum appears, add a random error
    if(distance>1&&abs(vx)<=0.08&&abs(vy)<=0.08)
        out(1) = -min(20,m/dt);  % go back steps (vx=-10->x=-1 if dt=0.1)
        out(2) = m*dt*(1+rand(1)); % move +y randomly
        m=m+1;
        disp("min");
    else
        out=speedLimit([v_store(i,1,t) v_store(1,2,t)],[vx vy],dt,detect_R);
    end

    % Update p_store and v_store (real values) for leader
    v_store(1,1,t+1)=out(1);
    v_store(1,2,t+1)=out(2);
    p_store(1,1,t+1) = p_store(1,1,t) + dt*v_store(1,1,t+1);
    p_store(1,2,t+1) = p_store(1,2,t) + dt*v_store(1,2,t+1);
    p_store(1,3,t+1) = atan2(v_store(1,2,t+1), v_store(1,1,t+1));

    %% Update pose with Kalman filter (leader)
    % Prediction
    pos_est_pred(1) = pos_est(1,t) + dt*vx_bar; % x_est(t) + dt*vx_bar(t)
    x_P1pred = x_P1 + dt*sigma_vel^2*dt';
    pos_est_pred(2) = pos_est(2,t) + dt*vy_bar; % y_est(t) + dt*vy_bar(t)
    y_P1pred = y_P1 + dt*sigma_vel^2*dt';
    
    % Update
    pos_meas(1,t+1) = p_store(1,1,t) + dt*v_store(1,1,t+1) + randn(1)*sigma_pos + mu_pos; % real x(t+1) with real u + measurement error 
    pos_meas(2,t+1) = p_store(1,2,t) + dt*v_store(1,2,t+1) + randn(1)*sigma_pos + mu_pos; % real y(t+1) with real u + measurement error 

    if (rand(1) <= Prob_sensor)
        H = 1;
        x_InnCov = H*x_P1pred*H' + sigma_pos^2;
        y_InnCov = H*y_P1pred*H' + sigma_pos^2;
        x_W = x_P1pred*H'*inv(x_InnCov);
        y_W = y_P1pred*H'*inv(y_InnCov);
        pos_est(1,t+1) = pos_est_pred(1) + x_W*(pos_meas(1,t+1) - H*pos_est_pred(1));
        pos_est(2,t+1) = pos_est_pred(2) + y_W*(pos_meas(2,t+1) - H*pos_est_pred(2));
        x_P1 = (eye(1) - x_W*H)*x_P1pred;
        y_P1 = (eye(1) - y_W*H)*y_P1pred;
    else
        pos_est(1,t+1) = pos_est_pred(1);
        x_P1 = x_P1pred;
        pos_est(2,t+1) = pos_est_pred(2);
        y_P1 = y_P1pred;
    end

    % Store the estimated covariance of the estimation error
    P1Store(1,t+1) = x_P1;
    P1PredStore(1,t+1) = x_P1pred;
    P1Store(2,t+1) = y_P1;
    P1PredStore(2,t+1) = y_P1pred;

    % Store vx_bar for the next iteration
    vx_bar = v_store(1,1,t+1) + randn(1)*sigma_vel + mu_vel; % vx_bar(t+1);
    vy_bar = v_store(1,2,t+1) + randn(1)*sigma_vel + mu_vel; % vx_bar(t+1);

    %% Calculate followers' velocity
    for i=2:n  %starts from 2, leader is out      
        sum_delta_x=0;
        sum_delta_y=0;
        sum_edge_weight=0;
        for j=1:n       % Linear Consensus control
                        % Condition Check: if A(i,j)==1: Checks if there is an edge between nodes i and j in the network represented by the adjacency matrix A.
                        % Edge Weight Calculation: Computes the edge weight w_ij based on a formula involving the Euclidean distance between the states of nodes i and j,
                        % Update Summations: Updates the sum of differences in x and y coordinates weighted by w_ij.
            if A(i,j)==1
                w_ij=2-exp(-((p_store(j,1,t-1)-p_store(i,1,t)-(delta_x(j)-delta_x(i)))^2+(p_store(j,2,t-1)-p_store(i,2,t)-(delta_y(j)-delta_y(i)))^2));%edge weighted calculation 
                sum_delta_x=sum_delta_x+A(i,j)*w_ij*((p_store(j,1,t-1)-p_store(i,1,t))-(delta_x(j)-delta_x(i)));                   
                sum_delta_y=sum_delta_y+A(i,j)*w_ij*((p_store(j,2,t-1)-p_store(i,2,t))-(delta_y(j)-delta_y(i)));
                sum_edge_weight=sum_edge_weight+w_ij;
            end
        end

        distance=sqrt(sum_delta_x^2+ sum_delta_y^2);
        th=atan2(sum_delta_y, sum_delta_x);
        if distance>d_max
            distance=d_max;
        end
        v_store(i,1,t+1)=K0*v_store(1,1,t)+gamma*distance*cos(th);     % ideal velocity without repulsion from obstalcle
        v_store(i,2,t+1)=K0*v_store(1,2,t)+gamma*distance*sin(th);

       % Consider repulsion among agents and store the pose to obs_pose
        kk=0;
        obs_pose=zeros(fol_num,2);
        for j=1:n
            if j~=i
                kk=kk+1;
                obs_pose(kk,1)=p_store(j,1,t);
                obs_pose(kk,2)=p_store(j,2,t);
            end
        end
        ob_pose=[obs_pose; ob_temp; ob_seabed];
        repulsion=computeRepulsion([p_store(i,1,t),p_store(i,2,t)],ob_pose,detect_R);        
        % put repulsion from obstacle on the robot velocity
        v_store(i,1,t+1)=v_store(i,1,t+1)+beta*repulsion(1);
        v_store(i,2,t+1)=v_store(i,2,t+1)+beta*repulsion(2);
    end

    %% Update pose (followers)

    % update the position and calculate error between prediction and
    % real position
    for i=2:n
        out=speedLimit([v_store(i,1,t) v_store(1,2,t)],[v_store(i,1,t+1) v_store(i,2,t+1)],dt,detect_R);
        v_store(i,1,t+1)=out(1);
        v_store(i,2,t+1)=out(2);
        p_store(i,1,t+1)=p_store(i,1,t)+dt*v_store(i,1,t+1);
        p_store(i,2,t+1)=p_store(i,2,t)+dt*v_store(i,2,t+1);
        p_store(i,3,t+1)=atan2(v_store(i,2,t+1),v_store(i,1,t+1));
    end

    %% Reaching goal
    
    now=[p_store(1,1,t+1),p_store(1,2,t+1)];

    % Exteroceptive measurements
    if norm(now-goal)<3
        
        % Update pose 
   
        pos_meas(1,t+1) = p_store(1,1,t+1) + randn(1)*sigma_pos_ext + mu_pos_ext; % real x(t+1) with real u + exteroceptive measurement error 
        pos_meas(2,t+1) = p_store(1,2,t+1) + randn(1)*sigma_pos_ext + mu_pos_ext; % real y(t+1) with real u + exteroceptive measurement error 
    
        H = 1;
        x_InnCov = H*x_P1pred*H' + sigma_pos^2;
        y_InnCov = H*y_P1pred*H' + sigma_pos^2;
        x_W = x_P1pred*H'*inv(x_InnCov);
        y_W = y_P1pred*H'*inv(y_InnCov);
        pos_est(1,t+1) = pos_est_pred(1) + x_W*(pos_meas(1,t+1) - H*pos_est_pred(1));
        pos_est(2,t+1) = pos_est_pred(2) + y_W*(pos_meas(2,t+1) - H*pos_est_pred(2));
        x_P1 = (eye(1) - x_W*H)*x_P1pred;
        y_P1 = (eye(1) - y_W*H)*y_P1pred;

        % Store the estimated covariance of the estimation error
        P1Store(1,t+1) = x_P1;
        P1PredStore(1,t+1) = x_P1pred;
        P1Store(2,t+1) = y_P1;
        P1PredStore(2,t+1) = y_P1pred;
    end

    if norm(now-goal)<1
        if anchor_num == length(myMap.anchors_x) && fin_goal_reached == 0
            disp('Final goal has been reached');
            fin_goal_reached = 1;
        end

        if anchor_num < length(myMap.anchors_x)
            string = ['Anchor number ', num2str(anchor_num), ' has been reached'];
            disp(string);
            anchor_num  = anchor_num + 1;
        end

        if fin_goal_reached
            stop=0;
            for i=2:n      % check if every agents reach final position ("zero" velocity)
                if abs(v_store(i,1,t))<=0.08 && abs(v_store(i,2,t))<=0.08
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
            m=1;    %reset local minima variable
            goal = [myMap.anchors_x(anchor_num) (myMap.anchors_y(anchor_num)+myMap.anchor_offset)];
        end
    end    
    
     %% ====Animation====
    hold off;
    ArrowLength=0.5;% 
    for j=1:n
        quiver(p_store(j,1,t+1),p_store(j,2,t+1),ArrowLength*cos(p_store(j,3,t+1)),ArrowLength*sin(p_store(j,3,t+1)),'k');
        hold on;
        if j==1    % used for draw leader in a different colour
            plotTriangle2d_V2(p_store(j,1,t+1), p_store(j,2,t+1), p_store(j,3,t+1),'r');
        else
            plotTriangle2d_V2(p_store(j,1,t+1), p_store(j,2,t+1), p_store(j,3,t+1),'g');
        end        
    end
    % draw the communication direction between two agents
    for i=1:n
        for j=1:n
            if A(i,j)==1
                drawArrow([p_store(j,1,t+1),p_store(j,2,t+1)],[p_store(i,1,t+1),p_store(i,2,t+1)], .5);
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
    drawnow;    
    
end
%% Draw diagram
color='mgbkrc';             % corresponding to 6 colors
type=[2,1,0.5,0.5,2,2];     % different line type
figure(2)                   % Draw the path record of formation 
for i=1:n
    plot(squeeze(p_store(i,1,2:t)),squeeze(p_store(i,2,2:t)),color(1,i),'LineWidth',2); % plot path
    hold on
end
for i=2:n
    plot(squeeze(p_store(i,1,2)),squeeze(p_store(i,2,2)),'bp','color',color(1,i),'LineWidth',1);    %plot followers' starting positions
    plot(squeeze(p_store(i,1,t)),squeeze(p_store(i,2,t)),'m^','color',color(1,i),'LineWidth',2);    %plot followers' final positions
    hold on
end
plot(squeeze(p_store(1,1,2)),squeeze(p_store(1,2,2)),'*','color',color(1,1),'LineWidth',1); %plot leader's starting point
plot(squeeze(p_store(1,1,t)),squeeze(p_store(1,2,t)),'o','color',color(1,1),'LineWidth',2); %plot leader's final point
hold on
if size(ob_pose)~=[0 0]
    plot(ob_pose(:,1),ob_pose(:,2),'Xk','LineWidth',2); %plot obstacles
    hold on;
end
grid on;
xlabel('x');
ylabel('y');
legend('Leader','Follower 1','Follower 2','Follower 3','Follower 4','Follower5');
xlabel('x(m)');
ylabel('y(m)');
title('Trajectory');

%% Plot KF results

figure(3), clf, hold on;
plot(squeeze(p_store(1,1,3:t)), squeeze(p_store(1,2,3:t)), 'b');
plot(pos_est(1,3:t), pos_est(2,3:t), 'r');
plot(pos_meas(1,3:t), pos_meas(2,3:t), 'g');
legend('Real', 'Estimated', 'Measured');
xlabel('t [s]');
ylabel('[m]');

figure(4), clf, hold on;
plot(time(3:t), P1Store(1,3:t), 'b');
plot(time(3:t), P1PredStore(1,3:t), 'r');
figure(5), clf, hold on;
plot(time(3:t), P1Store(2,3:t), 'g');
plot(time(3:t), P1PredStore(2,3:t), 'k');


