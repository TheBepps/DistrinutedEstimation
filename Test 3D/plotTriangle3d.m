
function plotTriangle3d(x, y, z, theta, phi, colour)
    % Define the vertices of the triangle
    triangle_vertices = [2, 0, 0; 0, -1, -1;  0, -1, 1; 0, 1, 1; 0, 1, -1; 0, -1, -1]*0.8;

    % Define rotation matrices for theta and phi
    R_theta = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1]; % Rotate around z-axis (theta)
    R_phi = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)]; % Rotate around x-axis (phi)

    % Combine rotation matrices (rotate phi first, then theta)
    R = R_phi * R_theta;

    % Apply rotation to the original matrix
    rotated_triangle = (triangle_vertices * R')';

    % Translate the rotated triangle to the position (x, y, z)
    translated_triangle = rotated_triangle' + [x, y, z];

    % Plot the triangle
    fill3(translated_triangle(2:5,1), translated_triangle(2:5,2), translated_triangle(2:5,3), 'k'); 
    fill3(translated_triangle(1:3,1), translated_triangle(1:3,2), translated_triangle(1:3,3), colour); 
    fill3(translated_triangle([1,3,4],1), translated_triangle([1,3,4],2), translated_triangle([1,3,4],3), colour); 
    fill3(translated_triangle([1,4,5],1), translated_triangle([1,4,5],2), translated_triangle([1,4,5],3), colour); 
    fill3(translated_triangle([1,2,5],1), translated_triangle([1,2,5],2), translated_triangle([1,2,5],3), colour);
end
