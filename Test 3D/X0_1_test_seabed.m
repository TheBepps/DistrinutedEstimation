myMap = struct();

myMap.dim=100;                                              % Map dimension
myMap.k = 0:0.5:myMap.dim;                                  % Time values from 0 to max dimension with a step of 0.1
myMap.profile = zeros(size(myMap.k,2));   % Initialize the hill profile
myMap.num_sin=10;                                           % number of sinusoids
myMap.amp_x=rand(1, myMap.num_sin)*2;                       % Random amplitudes between 0 and 2
myMap.amp_y=rand(1, myMap.num_sin)*2;                 
myMap.phase_x=rand(1, myMap.num_sin)* 2*pi;                 % Random phase shifts between 0 and 2*pi
myMap.phase_y=rand(1, myMap.num_sin)* 2*pi;          
myMap.freq_x=linspace(0.005, 0.03, myMap.num_sin)*0.6;       % Frequencies of the sinusoids
myMap.freq_y=linspace(0.005, 0.03, myMap.num_sin)*0.6; 
myMap.ground_zero = 40;   % zero level of the sea

myMap.anchors_x = [5, 20, 40, 65, 80, 95]; % Declare the anchor array
myMap.anchors_y = zeros(size(myMap.anchors_x));
myMap.anchor_offset=15;

x_profile=zeros(size(myMap.k));
y_profile=zeros(size(myMap.k));

for i = 1:myMap.num_sin
    x_profile = x_profile + myMap.amp_x(i) * sin(2*pi*myMap.freq_x(i)*myMap.k + myMap.phase_x(i));
    y_profile = y_profile + myMap.amp_y(i) * sin(2*pi*myMap.freq_y(i)*myMap.k + myMap.phase_y(i));
end

myMap.profile = x_profile' + y_profile;
myMap.profile = myMap.profile - myMap.ground_zero; %set level zero

[x, y] = meshgrid(myMap.k, myMap.k);

% Plot the 3D surface
figure;
surf(x, y, myMap.profile);
xlabel('X');
ylabel('Y');
zlabel('Height');
title('Seabed Profile');