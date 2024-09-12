function drawArrow3D(startpoint,endpoint,color,width) 

    % Calculate the direction vector
    direction = endpoint - startpoint;
    direction = direction / norm(direction); % Normalize the direction

    % Define arrowhead parameters
    arrowheadLength = 0.4 * norm(endpoint - startpoint); % Length of the arrowhead
    arrowheadWidth = 0.25 * norm(endpoint - startpoint); % Width of the arrowhead

    % Define the vertices of the arrowhead
    p1 = endpoint;
    p2 = endpoint - arrowheadLength * direction + arrowheadWidth * [-direction(2), direction(1), 0]; % Perpendicular to direction
    p3 = endpoint - arrowheadLength * direction - arrowheadWidth * [-direction(2), direction(1), 0]; % Perpendicular to direction

    % Plot the main arrow line
    x = [startpoint(1), endpoint(1)];
    y = [startpoint(2), endpoint(2)];
    z = [startpoint(3), endpoint(3)];
    plot3(x, y, z, 'Color', color, 'LineWidth', width);
    hold on;

    % Plot the arrowhead using fill3 to create a triangular face
    fill3([p1(1) p2(1) p3(1)], [p1(2) p2(2) p3(2)], [p1(3) p2(3) p3(3)], color);

end