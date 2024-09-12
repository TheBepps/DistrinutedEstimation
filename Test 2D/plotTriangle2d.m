function plotTriangle2d(x, y, theta)

    % Define the vertices of the triangle
    triangle_vertices = [1, 0; -1, 1; -1, -1; 1, 0];

    % Rotate the triangle by theta
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_triangle = (triangle_vertices * R')';

    % Translate the rotated triangle to the position (x, y)
    translated_triangle = rotated_triangle' + [x, y];

    % Plot the triangle
    fill(translated_triangle(:,1), translated_triangle(:,2), 'g'); % Filled triangle
end