clc;
clear;

% Time
Dt = 0.1;
time = 0:Dt:50;

% Parameters
d_max = 2; % above d_max, velocity remains const
beta = 10;  %object repulsion parameter
KN = 2; % velocity magnitude
detect_R = 3; % distance to detect obstacles
fin_goal_reached = 0;

% Define triangle initial state [x, y, theta, vx, vy]
s0 = [0; 20; 0; 0; 0];  % [x, y, theta, vx, vy]
sStore = zeros(length(s0), length(time));
sStore(:,1) = s0;

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
ob_num=50;
ob_temp=obstBuild(myMap, ob_num, detect_R);

% First goal
goal = [myMap.anchors_x(1), (myMap.anchors_y(1)+myMap.anchor_offset)];
anchor_num = 1;
 
%% Run
for t=1:length(time)-1
    % Calculate attraction from anchor and put it on leader velocity
    distance=sqrt((goal(1)-sStore(1,t))^2+(goal(2)-sStore(2,t))^2);
    th=atan2(goal(2)-sStore(2,t),goal(1)-sStore(1,t));
    if distance>d_max
        distance=d_max;
    end
    sStore(4,t+1)=KN*distance*cos(th);
    sStore(5,t+1)=KN*distance*sin(th);

    %% Calculate new triangle velocity   
    %create ob_seabed as avoidance object
    ob_seabed = [myMap.k(detect_R*2/3:detect_R*2/3:end); myMap.profile(detect_R*2/3:detect_R*2/3:end)]';%arr(x:y:end) start from x, take every y elements until end of vector arr
    % Concatenate ob_temp with the seabed
    ob_pose = [ob_temp; ob_seabed];
    repulsion=computeRepulsion([sStore(1,t),sStore(2,t)],ob_pose,detect_R);        
    % Put obstacle repulsion to leader
    sStore(4,t+1)=sStore(4,t+1)+beta*repulsion(1);
    sStore(5,t+1)=sStore(5,t+1)+beta*repulsion(2);

    % When the local minimum appears, add a random error
    if(distance>1&&abs(sStore(4,t+1))<=0.1&&abs(sStore(5,t+1))<=0.1)
        sStore(4,t+1)=-10;  %go back 1 steps (vx=-10->x=-1 if Dt=0.1)
        sStore(5,t+1)=(15-30*rand(1)); %move y +-1.5 randomly
    end

    %% Update pose
    sStore(1,t+1)=sStore(1,t)+Dt*sStore(4,t+1);
    sStore(2,t+1)=sStore(2,t)+Dt*sStore(5,t+1);
    sStore(3,t+1)=atan2(sStore(5,t+1),sStore(4,t+1));

     %% ====Animation====
    hold off;
    ArrowLength=0.5;% 
    quiver(sStore(1,t+1),sStore(2,t+1),ArrowLength*cos(sStore(3,t+1)),ArrowLength*sin(sStore(3,t+1)),'*k');
    hold on;
    if size(ob_temp)~=[0 0]
        plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);
        hold on;
    end
    seabedInitialization_2d_V2(myMap);
    plotTriangle2d(sStore(1,t), sStore(2,t), sStore(3,t));
    plot(goal(1),goal(2),'Xr','LineWidth',2)
    grid on;
    drawnow;    
    %% Reaching goal
    now=[sStore(1,t+1),sStore(2,t+1)];
    if norm(now-goal)<0.5
        if anchor_num < length(myMap.anchors_x)
            string = ['Anchor number ', num2str(anchor_num), ' has been reached'];
            disp(string);
        end
        if anchor_num == length(myMap.anchors_x) && fin_goal_reached == 0
            disp('Final goal has been reached');
            fin_goal_reached = 1;
            break;  %end for loop "Run section"
        end
        anchor_num  = anchor_num + 1;
        if anchor_num < length(myMap.anchors_x)+1
            goal = [myMap.anchors_x(anchor_num) (myMap.anchors_y(anchor_num)+myMap.anchor_offset)];
        end
    end    
end



