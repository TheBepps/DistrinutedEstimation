function plotPyramid3d(x, y, z, yaw, pitch, colour)

    % Define the vertices of the pyramid
%     pyramid_vertices = [2, 0, 0; 0, -1, -1;  0, -1, 1; 0, 1, 1; 0, 1, -1; 0, -1, -1]*0.8;
%     pyramid_vertices = [0, 2, 0; -1, 0, -1;  -1, 0, 1; 1, 0, 1; 1, 0, -1; -1, 0, -1]*0.8;
    pyramid_vertices = [0, 0, 2; -1, -1, 0;  1, -1, 0; 1, 1, 0; -1, 1, 0; -1, -1, 0]*0.8;

    % Define rotation matrices for yaw and pitch
    R_yaw = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1]; % Rotate around z-axis (yaw)
    R_pitch = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)]; % Rotate around y-axis along pyramid frame (pitch)

    % Combine rotation matrices (rotate pitch first, then yaw)
    R = R_yaw * R_pitch;

    % Apply rotation to the original matrix
    rotated_pyramid = (R*pyramid_vertices')';

    % Translate the rotated pyramid to the position (x, y, z)
    translated_pyramid = rotated_pyramid + [x, y, z];

    % translated_pyramid = pyramid_vertices + [x, y, z];

    % Plot the pyramid
    hold on;
    fill3(translated_pyramid(2:5,1), translated_pyramid(2:5,2), translated_pyramid(2:5,3), [0.5,0.5,0.5]); 
    fill3(translated_pyramid(1:3,1), translated_pyramid(1:3,2), translated_pyramid(1:3,3), colour); 
    fill3(translated_pyramid([1,3,4],1), translated_pyramid([1,3,4],2), translated_pyramid([1,3,4],3), colour); 
    fill3(translated_pyramid([1,4,5],1), translated_pyramid([1,4,5],2), translated_pyramid([1,4,5],3), colour); 
    fill3(translated_pyramid([1,2,5],1), translated_pyramid([1,2,5],2), translated_pyramid([1,2,5],3), colour);
end

