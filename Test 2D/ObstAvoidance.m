clc;
clear;

% Time
Dt = 0.1;
time = 0:Dt:30;

% Map dimension
m_dim=100;

% Parameters
d_max = 2; % above d_max, velocity remains const
beta = 10;
KN = 2; % velocity magnitude
detect_R = 5; % distance to detect obstacles
fin_goal_reached = 0;

% Declare the anchor array
anchors_x = [20, 40, 65, 80, 95];
anchor_offest=5; %NOT USED
goal_offset=5;

% Call seabedInitialization function with the anchor array
num_sin = 30; % Number of sinusoids
amp=rand(1, num_sin)*2; % Random amplitudes between 0 and 2
phase=rand(1, num_sin)* 2*pi; % Random phase shifts between 0 and 2*pi
[profile, anchors_y] = seabedInitialization_2d(anchors_x,amp,phase,num_sin,m_dim);

% First goal
goal = [anchors_x(1), (anchors_y(1)+goal_offset)];
anchor_num = 1;

% Define triangle initial state [x, y, theta, vx, vy]
s0 = [0; 20; 0; 0; 0];  % [x, y, theta, vx, vy]
sStore = zeros(length(s0), length(time));
sStore(:,1) = s0;
  
% Obstacles
ob_num=10;
ob_temp=obBuild(ob_num, m_dim, profile, detect_R,anchors_x);
 
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
    ob_pose=ob_temp;
    repulsion=compute_repulsion([sStore(1,t),sStore(2,t)],ob_pose,detect_R);        
    % Put obstacle repulsion to leader
    sStore(4,t+1)=sStore(4,t+1)+beta*repulsion(1);
    sStore(5,t+1)=sStore(5,t+1)+beta*repulsion(2);

    % When the local minimum appears, add a random error
    if(distance>1&&abs(sStore(4,t+1))<=0.1&&abs(sStore(5,t+1))<=0.1)
        sStore(4,t+1)=-1+2*rand(1);
        sStore(5,t+1)=-1+2*rand(1);
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
    seabedInitialization_2d(anchors_x,amp,phase,num_sin,m_dim);
    plotTriangle2d(sStore(1,t), sStore(2,t), sStore(3,t));
    plot(goal(1),goal(2),'Xr','LineWidth',2)
    grid on;
    drawnow;    
    %% Reaching goal
    now=[sStore(1,t+1),sStore(2,t+1)];
    if norm(now-goal)<0.5
        if anchor_num < length(anchors_x)
            string = ['Anchor number ', num2str(anchor_num), ' has been reached'];
            disp(string);
        end
        if anchor_num == length(anchors_x) && fin_goal_reached == 0
            disp('Final goal has been reached');
            fin_goal_reached = 1;
        end
        anchor_num  = anchor_num + 1;
        if anchor_num < length(anchors_x)+1
            goal = [anchors_x(anchor_num) (anchors_y(anchor_num)+goal_offset)];
        end
    end
    
end


function [profile, anchors_y] = seabedInitialization_2d(anchors_x,amp,phase,num_sin,m_dim)
    % Define the time range
    t = 0:0.1:m_dim; % Time values from 0 to 100 with a step of 0.1

    % Define parameters for the sinusoids
   
    freq = linspace(0.005, 0.1, num_sin)*0.6; % Frequencies of the sinusoids
   
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
    % Set x-axis and y-axis limits to 100
    xlim([0, m_dim]);
    ylim([floor(min(profile))-m_dim/2, m_dim/2]); 
    %xticks(0:10:100);
    %yticks(floor(min(profile)):10:100);  
    %axis equal;

    anchors_y = zeros(length(anchors_x));
    % Plot shapes using anchors_x array
    hold on;
    for i = 1:length(anchors_x)
        % Generate random color
        color = [min(1,0.01*anchors_x(i)),max(0,1-0.01*anchors_x(i)),max(1-0.01*anchors_x(i))];
        % Plot anchors
        plot(anchors_x(i), interp1(t, profile, anchors_x(i)), 'o', 'MarkerFaceColor', color, 'MarkerSize', 10);
        % anchors_y
        anchors_y(i) = interp1(t, profile, anchors_x(i));
    end
end


function [ repulsion] = compute_repulsion(robot_pose,obs_pose,detect_R)
    % compute the repulsion using artificial potential field method
    % obs_pose=[x1 y1;x2; y2;....]
    [M,N]=size(obs_pose);
    repulsion(1)=0; %x direction
    repulsion(2)=0; %y direction
    for i=1:M
        distance=sqrt((robot_pose(1)-obs_pose(i,1))^2+(robot_pose(2)-obs_pose(i,2))^2);
        if distance<=detect_R
            temp=(1/distance-1/detect_R)/(distance^3);
            repulsion(1)=repulsion(1)+temp*(robot_pose(1)-obs_pose(i,1));
            repulsion(2)=repulsion(2)+temp*(robot_pose(2)-obs_pose(i,2));
        end
    end
end

function [ob_temp]=obBuild(num,max,profile,detect_R,anchors_x)
    ob_temp=zeros(num,2);
    for i=1:num     %for each obstacle create a random (x,y) position
        flag=true;
        while flag  %check anchor-obstacle overlapping
            flag=false;
            ob_temp(i,1)=randi([0,max]);            
            for k=1:(size(anchors_x,2))
                if abs(ob_temp(i,1)-anchors_x(k))<detect_R  %inconsistent ancor and avoidance point
                    flag=true;
                end
            end
        end
        % after choosing X valid position
        ob_temp(i,2)=profile(ob_temp(i,1)*10)+randi([2,10]);  % obstacle Y position
    end
end

function plotTriangle2d(x, y, theta)

    % Define the vertices of the triangle
    triangle_vertices = [1, 0; -1, 1; -1, -1; 1, 0]*2;

    % Rotate the triangle by theta
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_triangle = (triangle_vertices * R')';

    % Translate the rotated triangle to the position (x, y)
    translated_triangle = rotated_triangle' + [x, y];

    % Plot the triangle
    fill(translated_triangle(:,1), translated_triangle(:,2), 'g'); % Filled triangle
end
