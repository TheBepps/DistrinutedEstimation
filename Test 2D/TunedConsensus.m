clc;
clear;

% Time
Dt = 0.1;
time = 0:Dt:100;
countMax=size(time);

%% Parameters
d_max = 2;          % above d_max, velocity remains const
beta =20;           %object repulsion parameter
KN = 0.8;           % leader's velocity coeff. KN<K0 in order to "speed up" followers' movements
detect_R = 4;       % distance to detect obstacles
fin_goal_reached = 0;

% consensus
fol_num=5;          % 5 followers
N=fol_num+1;        % 5 followers and 1 leader
gama=1.2;           %t+1 followers velocity multiplier (useful for maintaining relative position)            
K0=1;               % followers' velocity coeff. Be aware to increas more than 1, velocity & obstales' repulsion are correlated 
A=[0 1 0 1 0 1;     % a(ij)
   1 0 0 0 1 1;
   0 0 0 1 0 1;
   1 0 1 0 1 0;
   0 1 0 1 0 0;
   1 1 1 0 0 0];
% consensus relative position 
delta_x=[0 1.5 -3 -1.5 1.5 -1.5]*2.5;    
delta_y=[0 1.5 0 -1.5 -1.5 1.5]*2.5;  %relative position between leader and follwers
%Initial position matrix
s0=zeros(N,5); % [x, y, theta, vx, vy] for N followers
for i=1:N
    s0(i,1)=2;
    s0(i,2)=20+detect_R/2*i;
%     s0(i,1)=10+delta_x(:,i);  %exact relative position
%     s0(i,2)=20+delta_y(:,i);
end
% Define triangles initial state 
sStore=zeros(length(s0(:,1)), length(s0(1,:)), length(time));   %rows -> each follower
                                                                %coloums -> [x, y, theta, vx, vy] 
                                                                %depth -> time history
sStore(:,:,2) = s0;%starting from time=2 

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
edge_w=zeros([fol_num countMax]);
sum_weight=[0;0;0;0;0;0];

%% Run
for t=2:length(time)-1
    % Calculate attraction from anchor and put it on leader velocity (first triangle)
    distance=sqrt((goal(1)-sStore(1,1,t))^2+(goal(2)-sStore(1,2,t))^2);
    th=atan2(goal(2)-sStore(1,2,t),goal(1)-sStore(1,1,t));
    if distance>d_max
        distance=d_max;
    end
    sStore(1,4,t+1)=KN*distance*cos(th);
    sStore(1,5,t+1)=KN*distance*sin(th);

    %% Calculate leader's new velocity   
    %create ob_seabed as avoidance object
    ob_seabed = [myMap.k(detect_R:detect_R:end); myMap.profile(detect_R:detect_R:end)]';%arr(x:y:end) start from x, take every y elements until end of vector arr
    % Concatenate ob_temp with the seabed
    ob_pose = [ob_temp; ob_seabed];
    repulsion=computeRepulsion([sStore(1,1,t),sStore(1,2,t)],ob_pose,detect_R);        
    % Put obstacle repulsion to leader
    sStore(1,4,t+1)=sStore(1,4,t+1)+beta/2*repulsion(1);    %beta/2 different from followers
    sStore(1,5,t+1)=sStore(1,5,t+1)+beta/2*repulsion(2);

    % When the local minimum appears, add a random error
    if(distance>1&&abs(sStore(1,4,t+1))<=0.1&&abs(sStore(1,5,t+1))<=0.1)
        sStore(1,4,t+1)=-10;  %go back 1 steps (vx=-10->x=-1 if Dt=0.1)
        sStore(1,5,t+1)=(15-30*rand(1)); %move y +-1.5 randomly
    end

     %% Calculate followers' velocity
        for i=2:N  %starts from 2, leader is out      
            sum_delta_x=0;
            sum_delta_y=0;
            sum_edge_weight=0;
            for j=1:N       %%Linear Consensus control
%                 Condition Check: if A(i,j)==1: Checks if there is an edge between nodes i and j in the network represented by the adjacency matrix A.
%                 Edge Weight Calculation:Computes the edge weight w_ij based on a formula involving the Euclidean distance between the states of nodes i and j,
%                 Update Summations: Updates the sum of differences in x and y coordinates weighted by w_ij.
                if A(i,j)==1
                    w_ij=2-exp(-((sStore(j,1,t-1)-sStore(i,1,t)-(delta_x(j)-delta_x(i)))^2+(sStore(j,2,t-1)-sStore(i,2,t)-(delta_y(j)-delta_y(i)))^2));%edge weighted calculation 
                    sum_delta_x=sum_delta_x+A(i,j)*w_ij*((sStore(j,1,t-1)-sStore(i,1,t))-(delta_x(j)-delta_x(i)));                   
                    sum_delta_y=sum_delta_y+A(i,j)*w_ij*((sStore(j,2,t-1)-sStore(i,2,t))-(delta_y(j)-delta_y(i)));
                    sum_edge_weight=sum_edge_weight+w_ij;
                end
            end
            %% sum_weight NOT USED
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
            sStore(i,4,t+1)=K0*sStore(1,4,t)+gama*distance*cos(th); % ideal velocity without repulsion from obstalcle
            sStore(i,5,t+1)=K0*sStore(1,5,t)+gama*distance*sin(th);
            
            %% NEED VALIDATION
            %%non serve a niente controllare max velocità dei followers
            %%tanto lo fa gia nel for di update pose sia per leader che followers 
%             out=confine([sStore(i,4,t) sStore(1,5,t)],[sStore(i,4,t+1) sStore(i,5,t+1)],Dt);
% %             out=[V_x(i,k+1) V_y(i,k+1)];
%             sStore(i,4,t+1)=out(1);
%             sStore(i,5,t+1)=out(2);

           %%%Consider repulsion among agents and store the pose to obs_pose
            kk=0;
            obs_pose=zeros(fol_num^2-fol_num,2);
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=sStore(j,1,t);
                    obs_pose(kk,2)=sStore(j,2,t);
                end
            end
            ob_pose=[obs_pose; ob_temp; ob_seabed];
            repulsion=computeRepulsion([sStore(i,1,t),sStore(i,2,t)],ob_pose,detect_R);        
            % put repulsion from obstacle on the robot velocity
            sStore(i,4,t+1)=sStore(i,4,t+1)+beta*repulsion(1);
            sStore(i,5,t+1)=sStore(i,5,t+1)+beta*repulsion(2);
 
        end

    %% Update pose
        % update the position and calculate error between prediction and
        % real position
        for i=1:N
            out=confine([sStore(i,4,t) sStore(1,5,t)],[sStore(i,4,t+1) sStore(i,5,t+1)],Dt);
            sStore(i,4,t+1)=out(1);
            sStore(i,5,t+1)=out(2);
            sStore(i,1,t+1)=sStore(i,1,t)+Dt*sStore(i,4,t+1);
            sStore(i,2,t+1)=sStore(i,2,t)+Dt*sStore(i,5,t+1);
            sStore(i,3,t+1)=atan2(sStore(i,5,t+1),sStore(i,4,t+1));
        end
     %% ====Animation====
    hold off;
    ArrowLength=0.5;% 
    for j=1:N
        quiver(sStore(j,1,t+1),sStore(j,2,t+1),ArrowLength*cos(sStore(j,3,t+1)),ArrowLength*sin(sStore(j,3,t+1)),'*k');
        hold on;
        if j==1    %USE to draw leader in a different colour
            plotTriangle2d_V2(sStore(j,1,t+1), sStore(j,2,t+1), sStore(j,3,t+1),'r');
        else
            plotTriangle2d_V2(sStore(j,1,t+1), sStore(j,2,t+1), sStore(j,3,t+1),'g');
        end        
    end
    % draw the communication direction between two agents
    for i=1:N
        for j=1:N
            if A(i,j)==1
                draw_arrow([sStore(j,1,t+1),sStore(j,2,t+1)],[sStore(i,1,t+1),sStore(i,2,t+1)], .5);
                hold on;
            end
        end
    end
    %draw obstacles
    if size(ob_temp)~=[0 0]
        plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);
        hold on;
    end
    seabedInitialization_2d_V2(myMap);
    plot(goal(1),goal(2),'Xr','LineWidth',2)
    grid on;
    drawnow;    
    %% Reaching goal
    now=[sStore(1,1,t+1),sStore(1,2,t+1)];
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
            for i=2:N %check if every agents reach final position ("zero" velocity)
                if abs(sStore(i,4,t))<=0.05 && abs(sStore(i,5,t))<=0.05
                    stop=1;
                else
                    stop=0;
                    break; %someone moves so we don't need to check others
                end
            end
            if stop
                disp('Everyone reaches the final position');
                break;  %end for loop "Run section"                
            end
        end
        
        if anchor_num < length(myMap.anchors_x)+1
            goal = [myMap.anchors_x(anchor_num) (myMap.anchors_y(anchor_num)+myMap.anchor_offset)];
        end
    end    
end
%% Draw diagram
color='mgbkrc'; %%%corresponding to 6 colors
type=[2,1,0.5,0.5,2,2];%%%different line type
figure                               % Draw the path record of formation 
for i=1:N
    plot(squeeze(sStore(i,1,2:t)),squeeze(sStore(i,2,2:t)),color(1,i),'LineWidth',2);
    hold on
end
for i=2:N
    plot(squeeze(sStore(i,1,1)),squeeze(sStore(i,2,1)),'bp','color',color(1,i),'LineWidth',1);
    hold on
end
plot(squeeze(sStore(1,1,1)),squeeze(sStore(1,2,1)),'*','color',color(1,1),'LineWidth',1);
hold on
for i=2:N
    plot(squeeze(sStore(i,1,t)),squeeze(sStore(i,2,t)),'m^','color',color(1,i),'LineWidth',2);
    hold on
end
plot(squeeze(sStore(1,1,t)),squeeze(sStore(1,2,t)),'o','color',color(1,1),'LineWidth',2);
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


function [ next ] = confine(current,next,dt)
    %%%current=[v_x v_y];
    %%%%Kinematic=[ x m/s],y [m/s],x [m/ss],y [m/ss]]
    Kinematic=[2;2;4;4];     
    %% limition on speed x
    delta_x=next(1)-current(1);
    if delta_x>=0
        next(1)=min(current(1)+delta_x,current(1)+Kinematic(3)*dt);
    else
        next(1)=max(current(1)+delta_x,current(1)-Kinematic(3)*dt);
    end
    if next(1)>=0
        next(1)=min(next(1),Kinematic(1));
    else
        next(1)=max(next(1),-Kinematic(1));
    end
    %% limition on speed y
    delta_y=next(2)-current(2);
    if delta_y>=0
        next(2)=min(current(2)+delta_y,current(2)+Kinematic(4)*dt);
    else
        next(2)=max(current(2)+delta_y,current(2)-Kinematic(4)*dt);
    end
    if next(2)>=0
        next(2)=min(next(2),Kinematic(2));
    else
        next(2)=max(next(2),-Kinematic(2));
    end
end

function [ output_args ] = draw_arrow(startpoint,endpoint,headsize) 
% draw the communication direction between two agents
% accepts two [x y] coords and one double headsize 

v1 = headsize*(startpoint-endpoint)/sqrt((startpoint(1)-endpoint(1))^2+(startpoint(2)-endpoint(2))^2);
theta = 22.5*pi/180; 
theta1 = -1*22.5*pi/180; 
rotMatrix = [cos(theta)  -sin(theta) ; sin(theta)  cos(theta)];
rotMatrix1 = [cos(theta1)  -sin(theta1) ; sin(theta1)  cos(theta1)];  

v2 = v1*rotMatrix; 
v3 = v1*rotMatrix1;
x1 = endpoint+2*v1;
x2 = x1 + v2;
x3 = x1 + v3;

fill([x1(1) x2(1) x3(1)],[x1(2) x2(2) x3(2)],[0 0 0]);% this fills the arrowhead (black) 

plot([startpoint(1) endpoint(1)],[startpoint(2) endpoint(2)],'linewidth',0.5,'color',[0 0 0]);

end


