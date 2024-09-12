clc;
clear;

% Time
dt = 0.18;
time = 0:dt:150;
count_max=size(time);

%% Parameters
m=1;    %local minima detection
d_max = 2;          % above d_max, velocity remains const
beta =15;           % object repulsion parameter
Kn = 0.8;           % leader's velocity coeff. Kn<K0 in order to "speed up" followers' movements
detect_R = 4;       % distance to detect obstacles (affects max velocity in speedLimit)
fin_goal_reached = 0;

% consensus
fol_num=5;          % 5 followers
n=fol_num+1;        % 5 followers and 1 leader
gamma=1;            % t+1 followers velocity multiplier (useful for maintaining relative position)            
K0=1;               % followers' velocity coeff. Be aware to increas more than 1, velocity & obstales' repulsion are correlated 
A=[0 1 0 1 0 1;     % a(ij)
   1 0 0 0 1 1;
   0 0 0 1 0 1;
   1 0 1 0 1 0;
   0 1 0 1 0 0;
   1 1 1 0 0 0];
% consensus relative position 
delta_x=[0 1.5 -3 -1.5 1.5 -1.5]*2.5;  
delta_y=[0 1.5 0 -1.5 -1.5 1.5]*2.5;  % relative position between leader and follwers
% Initial position matrix
s_0=zeros(n,5); % [x, y, theta, vx, vy] for n followers
for i=1:n
    s_0(i,1)=4*rand(1);
    s_0(i,2)=25+detect_R/2*i*rand(1);
%     s_0(i,1)=10+delta_x(:,i);  % exact relative position
%     s_0(i,2)=20+delta_y(:,i);
end
% Define triangles initial state 
s_store=zeros(length(s_0(:,1)), length(s_0(1,:)), length(time));    %rows -> each follower
                                                                    %coloums -> [x, y, theta, vx, vy] 
                                                                    %depth -> time history
s_store(:,:,2) = s_0; % starting from time=2 

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
ob_num=30;
ob_temp=obstBuild(myMap, ob_num, detect_R);

% First goal
goal = [myMap.anchors_x(1), (myMap.anchors_y(1)+myMap.anchor_offset)];
anchor_num = 1;

%% edge weighted matrix
edge_w=zeros([fol_num count_max]);
sum_weight=[0;0;0;0;0;0];

%% Run
for t=2:length(time)-1
    % Calculate attraction from anchor and put it on leader velocity (first triangle)
    distance=sqrt((goal(1)-s_store(1,1,t))^2+(goal(2)-s_store(1,2,t))^2);
    th=atan2(goal(2)-s_store(1,2,t),goal(1)-s_store(1,1,t));
    if distance>d_max
        distance=d_max;
    end
    s_store(1,4,t+1)=Kn*distance*cos(th);
    s_store(1,5,t+1)=Kn*distance*sin(th);

    %% Calculate leader's new velocity   
    % create ob_seabed as avoidance objects
    ob_seabed = [myMap.k(detect_R:detect_R:end); myMap.profile(detect_R:detect_R:end)]';% arr(x:y:end) start from x, take every y elements until end of vector arr
    % Concatenate ob_temp with the seabed
    ob_pose = [ob_temp; ob_seabed];
    repulsion=computeRepulsion([s_store(1,1,t),s_store(1,2,t)],ob_pose,detect_R);        
    % Put obstacle repulsion to leader
    s_store(1,4,t+1)=s_store(1,4,t+1)+beta/3.5*repulsion(1);    %beta/3 different from followers
    s_store(1,5,t+1)=s_store(1,5,t+1)+beta/3.5*repulsion(2);

     %% Calculate followers' velocity
    for i=2:n  %starts from 2, leader is out      
        sum_delta_x=0;
        sum_delta_y=0;
        sum_edge_weight=0;
        for j=1:n       % Linear Consensus control
                        % Condition Check: if A(i,j)==1: Checks if there is an edge between nodes i and j in the network represented by the adjacency matrix A.
                        % Edge Weight Calculation:Computes the edge weight w_ij based on a formula involving the Euclidean distance between the states of nodes i and j,
                        % Update Summations: Updates the sum of differences in x and y coordinates weighted by w_ij.
            if A(i,j)==1
                w_ij=2-exp(-((s_store(j,1,t-1)-s_store(i,1,t)-(delta_x(j)-delta_x(i)))^2+(s_store(j,2,t-1)-s_store(i,2,t)-(delta_y(j)-delta_y(i)))^2));%edge weighted calculation 
                sum_delta_x=sum_delta_x+A(i,j)*w_ij*((s_store(j,1,t-1)-s_store(i,1,t))-(delta_x(j)-delta_x(i)));                   
                sum_delta_y=sum_delta_y+A(i,j)*w_ij*((s_store(j,2,t-1)-s_store(i,2,t))-(delta_y(j)-delta_y(i)));
                sum_edge_weight=sum_edge_weight+w_ij;
            end
        end
        %% sum_weight nOT USED
%             edge_w(i,t)=sum_edge_weight;
%             % Store the weight error into sum_weight matrix
%             if edge_w(i,t)>edge_w(i,t-1)&&t>2
%                 sum_weight(i,t)=sum_weight(i,t-1)+abs(edge_w(i,t)-edge_w(i,t-1));
%             else
%                 sum_weight(i,t)=sum_weight(i,t-1);
%             end
%             if mod(t,100)==1 % reset matrix to 0 after period 10s
%                 sum_weight(i,t)=0;
%             end

        distance=sqrt(sum_delta_x^2+ sum_delta_y^2);
        th=atan2(sum_delta_y, sum_delta_x);
        if distance>d_max
            distance=d_max;
        end
        s_store(i,4,t+1)=K0*s_store(1,4,t)+gamma*distance*cos(th);     % ideal velocity without repulsion from obstalcle
        s_store(i,5,t+1)=K0*s_store(1,5,t)+gamma*distance*sin(th);
        
        %% NEED VALIDATION
        %%non serve a niente controllare max velocità dei followers
        %%tanto lo fa gia nel for di update pose sia per leader che followers 
%             out=speedLimit([s_store(i,4,t) s_store(1,5,t)],[s_store(i,4,t+1) s_store(i,5,t+1)],dt);
% %             out=[V_x(i,k+1) V_y(i,k+1)];
%             s_store(i,4,t+1)=out(1);
%             s_store(i,5,t+1)=out(2);

       %Consider repulsion among agents and store the pose to obs_pose
        kk=0;
        obs_pose=zeros(fol_num,2);
        for j=1:n
            if j~=i
                kk=kk+1;
                obs_pose(kk,1)=s_store(j,1,t);
                obs_pose(kk,2)=s_store(j,2,t);
            end
        end
        ob_pose=[obs_pose; ob_temp; ob_seabed];
        repulsion=computeRepulsion([s_store(i,1,t),s_store(i,2,t)],ob_pose,detect_R);        
        % put repulsion from obstacle on the robot velocity
        s_store(i,4,t+1)=s_store(i,4,t+1)+beta*repulsion(1);
        s_store(i,5,t+1)=s_store(i,5,t+1)+beta*repulsion(2);

    end

    %% Update pose
    % update the position and calculate error between prediction and
    % real position
    for i=1:n
        out=speedLimit([s_store(i,4,t) s_store(1,5,t)],[s_store(i,4,t+1) s_store(i,5,t+1)],dt,detect_R);
        s_store(i,4,t+1)=out(1);
        s_store(i,5,t+1)=out(2);
        s_store(i,1,t+1)=s_store(i,1,t)+dt*s_store(i,4,t+1);
        s_store(i,2,t+1)=s_store(i,2,t)+dt*s_store(i,5,t+1);
        s_store(i,3,t+1)=atan2(s_store(i,5,t+1),s_store(i,4,t+1));
    end
    % When the local minimum appears, add a random error
    %after update pose to avoid speed limitation
    distance=sqrt((goal(1)-s_store(1,1,t))^2+(goal(2)-s_store(1,2,t))^2);
    if(distance>1&&abs(s_store(1,4,t+1))<=0.05&&abs(s_store(1,5,t+1))<=0.05)
        s_store(1,4,t+1)=-m/dt;  %go back steps (vx=-10->x=-1 if dt=0.1)
        s_store(1,5,t+1)=m/2*dt+m/dt*rand(1); %move +y randomly
        s_store(1,1,t+1)=s_store(1,1,t)+dt*s_store(1,4,t+1);
        s_store(1,2,t+1)=s_store(1,2,t)+dt*s_store(1,5,t+1);
        m=m+1;
    end
     %% ====Animation====
    hold off;
    ArrowLength=0.5;% 
    for j=1:n
        quiver(s_store(j,1,t+1),s_store(j,2,t+1),ArrowLength*cos(s_store(j,3,t+1)),ArrowLength*sin(s_store(j,3,t+1)),'*k');
        hold on;
        if j==1    % used for draw leader in a different colour
            plotTriangle2d_V2(s_store(j,1,t+1), s_store(j,2,t+1), s_store(j,3,t+1),'r');
        else
            plotTriangle2d_V2(s_store(j,1,t+1), s_store(j,2,t+1), s_store(j,3,t+1),'g');
        end        
    end
    % draw the communication direction between two agents
    for i=1:n
        for j=1:n
            if A(i,j)==1
                drawArrow([s_store(j,1,t+1),s_store(j,2,t+1)],[s_store(i,1,t+1),s_store(i,2,t+1)], .5);
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
    %% Reaching goal
    now=[s_store(1,1,t+1),s_store(1,2,t+1)];
    if norm(now-goal)<0.5
        if anchor_num < length(myMap.anchors_x)
            string = ['Anchor number ', num2str(anchor_num), ' has been reached'];
            disp(string);
            anchor_num  = anchor_num + 1;
        end

        if anchor_num == length(myMap.anchors_x) && fin_goal_reached == 0
            disp('Final goal has been reached');
            fin_goal_reached = 1;
        end

        if fin_goal_reached
            stop=0;
            for i=2:n      % check if every agents reach final position ("zero" velocity)
                if abs(s_store(i,4,t))<=0.08 && abs(s_store(i,5,t))<=0.08
                    stop=1;
                else
                    stop=0;
                    break;  % someone moves so we don't need to check others
                end
            end
            if stop
                disp('Everyone reaches the final position');
                break;      % end for loop "Run section"                
            end
        end
        
        if anchor_num < length(myMap.anchors_x)+1
            m=1;
            goal = [myMap.anchors_x(anchor_num) (myMap.anchors_y(anchor_num)+myMap.anchor_offset)];
        end
    end    
end
%% Draw diagram
color='mgbkrc';             % corresponding to 6 colors
type=[2,1,0.5,0.5,2,2];     % different line type
figure                      % Draw the path record of formation 
for i=1:n
    plot(squeeze(s_store(i,1,2:t)),squeeze(s_store(i,2,2:t)),color(1,i),'LineWidth',2);
    hold on
end
for i=2:n
    plot(squeeze(s_store(i,1,1)),squeeze(s_store(i,2,1)),'bp','color',color(1,i),'LineWidth',1);
    hold on
end
plot(squeeze(s_store(1,1,1)),squeeze(s_store(1,2,1)),'*','color',color(1,1),'LineWidth',1);
hold on
for i=2:n
    plot(squeeze(s_store(i,1,t)),squeeze(s_store(i,2,t)),'m^','color',color(1,i),'LineWidth',2);
    hold on
end
plot(squeeze(s_store(1,1,t)),squeeze(s_store(1,2,t)),'o','color',color(1,1),'LineWidth',2);
hold on
if size(ob_pose)~=[0 0]
    plot(ob_pose(:,1),ob_pose(:,2),'Xk','LineWidth',2);hold on;
end
grid on;
xlabel('x');
ylabel('y');
legend('Leader','Follower 1','Follower 2','Follower 3','Follower 4','Follower5');
xlabel('x(m)');
ylabel('y(m)');
title('Formation Consensus');


