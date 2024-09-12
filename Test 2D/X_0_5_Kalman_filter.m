clc;
clear;
addpath("..\DistributedEstimation\Test 2D\")

% Parameters
m=1;    %local minima detection
d_max = 2; % above d_max, velocity remains const
beta = 15;
KN = 0.8; % velocity magnitude
detect_R = 4; % distance to detect obstacles
fin_goal_reached = 0;

% Time
Dt = 0.18;
time = 0:Dt:150;

%% Uncertainty
Prob_sensor = 0.3;
pStore = zeros(3, length(time)); % states [x, y, theta]
vStore = zeros(2, length(time)); % velocities [vx, vy]
% Velocity errors
mu_vx = 0;
sigma_vx = 0.5;
% Position measurements
mu_x = 0;
sigma_x = 5;
% Initial values
pStore(1,1) = 0;
pStore(2,1) = 20;
pStore(3,1) = 0;
vStore(1,1) = 0;
vStore(2,1) = 0;
vx_bar_old = 0 + randn(1)*sigma_vx + mu_vx;

% Kalman filter
x_est = zeros(1, length(time)); % estimated x
x_meas = zeros(1, length(time)); % x measurements
P1 = 5;
P1Store = zeros(1, length(time));
P1PredStore = zeros(1, length(time));

%% Define the structure of MAP
myMap = struct();

myMap.dim=100;  % Map dimension
myMap.k = 0:0.1:myMap.dim; % Time values from 0 to max dimension with a step of 0.1
myMap.profile = zeros(size(myMap.k)); % Initialize the hill profile
myMap.num_sin=30;   % Number of sinusoids
myMap.amp=rand(1, myMap.num_sin)*2;   % Random amplitudes between 0 and 2
myMap.phase=rand(1, myMap.num_sin)* 2*pi; % Random phase shifts between 0 and 2*pi
myMap.freq=linspace(0.005, 0.1, myMap.num_sin)*0.6; % Frequencies of the sinusoids

myMap.anchors_x = [20, 40, 65, 80, 95]; % Declare the anchor array
myMap.anchors_y = zeros(size(myMap.anchors_x));
myMap.anchor_offset=10;

% Generate the hill profile by summing sinusoids
for i = 1:myMap.num_sin
    myMap.profile = myMap.profile + myMap.amp(i) * sin(2*pi*myMap.freq(i)*myMap.k + myMap.phase(i));
end
% anchors_y
for i = 1:length(myMap.anchors_x)
    myMap.anchors_y(i) = interp1(myMap.k, myMap.profile, myMap.anchors_x(i));
end
  
% Obstacles
ob_num=30;
ob_temp=obstBuild(myMap, ob_num, detect_R);

% First goal
goal = [myMap.anchors_x(1), (myMap.anchors_y(1)+myMap.anchor_offset)];
anchor_num = 1;
 
%% Run
for t=1:length(time)-1

    % Calculate attraction from anchor to compute leader velocity
    distance=sqrt((goal(1)-x_est(t))^2+(goal(2)-pStore(2,t))^2); % x_est(t) because leader doesn't know real x (pStore(1,t))
    th=atan2(goal(2)-pStore(2,t),goal(1)-x_est(t));
    if distance>d_max
        distance=d_max;
    end
    vx = KN*distance*cos(th);
    vy = KN*distance*sin(th);

    % Calculate new triangle velocity with obstacles   
    ob_seabed = [myMap.k(detect_R:detect_R:end); myMap.profile(detect_R:detect_R:end)]';% arr(x:y:end) start from x, take every y elements until end of vector arr
    % Concatenate ob_temp with the extra rows
    ob_pose = [ob_temp; ob_seabed];
    repulsion=computeRepulsion([pStore(1,t),pStore(2,t)],ob_pose,detect_R);        
    % Put obstacle repulsion to leader
    vx = vx + beta/3.5*repulsion(1);
    vy = vy + beta/3.5*repulsion(2);

    % When the local minimum appears, add a random error
    if(distance>1&&abs(vx)<=0.08&&abs(vy)<=0.08)
        vx = -m/Dt;  % go back steps (vx=-10->x=-1 if Dt=0.1)
        vy = m/2*Dt+m/Dt*rand(1); % move +y randomly
        m=m+1;
    end

    % Update pStore and vStore (real values)
    pStore(1,t+1) = pStore(1,t) + Dt*vx;
    pStore(2,t+1) = pStore(2,t) + Dt*vy;
    pStore(3,t+1) = atan2(vy, vx);
    vStore(1,t+1) = vx;
    vStore(2,t+1) = vy;

    % Update vx_bar (vx(t+1) + error)
    vx_bar_new = vx + randn(1)*sigma_vx + mu_vx; % vx_bar(t+1)

    %% Update pose with Kalman filter (only x)
    
    %OTTIMIZZAZIONE: NN E' IL CASO DI AGGIORNARE VX_BAR DOPO PREDICTION E
    %QUINDI NON SERVE PIU ALLOCARE VX_BAR_OLD PER LA PROSSIMA ITERAZIONE???
    %TANTO E' GIA INIZIALIZZATA ALL'INIZIO

    % Prediction
    x_est_pred = x_est(t) + Dt*vx_bar_old; % x_est(t) + Dt*vx_bar(t)
    P1pred = P1 + Dt*sigma_vx^2*Dt';
    
    % Update
    x_meas(t+1) = pStore(1,t) + Dt*vx + randn(1)*sigma_x + mu_x; % real x(t+1) with real u + measurement error 
    if (rand(1) <= Prob_sensor)
        H = 1;
        InnCov = H*P1pred*H' + sigma_x^2;
        W = P1pred*H'*inv(InnCov);
        x_est(t+1) = x_est_pred + W*(x_meas(t+1) - H*x_est_pred);
        P1 = (eye(1) - W*H)*P1pred;
    else
        x_est(t+1) = x_est_pred;
        P1 = P1pred;
    end
    
    % Store the estimated covariance of the estimation error
    P1Store(t+1) = P1;
    P1PredStore(t+1) = P1pred;

    % Store vx_bar for the next iteration
    vx_bar_old = vx_bar_new;

     %% ====Animation====
    hold off;
    ArrowLength=0.5;% 
    quiver(pStore(1,t+1),pStore(2,t+1),ArrowLength*cos(pStore(3,t+1)),ArrowLength*sin(pStore(3,t+1)),'k');
    hold on;
    if size(ob_temp)~=[0 0]
        plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);
        hold on;
    end
    seabedInitialization_2d_V2(myMap);
    plotTriangle2d(pStore(1,t), pStore(2,t), pStore(3,t));
    plot(goal(1),goal(2),'Xr','LineWidth',2)
    grid on;
    drawnow;   

    %% Reaching goal
    now=[x_est(t+1),pStore(2,t+1)];
    if norm(now-goal)<0.5
        if anchor_num < length(myMap.anchors_x)
            string = ['Anchor number ', num2str(anchor_num), ' has been reached'];
            disp(string);
            m=1;    %reset local minima variable
        end
        if anchor_num == length(myMap.anchors_x) && fin_goal_reached == 0
            disp('Final goal has been reached');
            fin_goal_reached = 1;
            break;  % end for loop "Run"
        end
        anchor_num  = anchor_num + 1;
        if anchor_num < length(myMap.anchors_x)+1
            goal = [myMap.anchors_x(anchor_num) (myMap.anchors_y(anchor_num)+myMap.anchor_offset)];
        end
    end    
end

%% Plot

figure(1), clf, hold on;
plot(time, pStore(1,:), 'b');
plot(time, x_est, 'r');
plot(time, x_meas, 'g');
legend('Real x', 'Estimated x', 'Measured x');
xlabel('t [s]');
ylabel('[m]');

figure(2), clf, hold on;
plot(time, P1Store, 'b');
plot(time, P1PredStore, 'r');
