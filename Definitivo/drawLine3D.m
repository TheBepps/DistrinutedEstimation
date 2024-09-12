function drawLine3D(startpoint,endpoint,color,width) 

    x = [startpoint(:, 1), endpoint(:, 1)]';
    y = [startpoint(:, 2), endpoint(:, 2)]';
    z = [startpoint(:, 3), endpoint(:, 3)]';
    
    % Plot the line
    plot3(x, y, z, 'Color', color, 'LineWidth', width);
    hold on;
end