function map = drawMap(map)
    % Plot the profile
    [x, y] = meshgrid(map.k, map.k);
    surf(x, y, map.profile);
    xlabel('X');
    ylabel('Y');
    zlabel('Height');
    title('Seabed Profile');
    grid on;
    % Set x-axis and y-axis limits to 100
%     xlim([0, map.dim]);
%     ylim([0, map.dim]);
%     zlim([floor(min(min(map.profile)))-map.dim/2, map.dim/2]); 

    % Plot shapes using anchors_x array
    hold on;
    plot3(map.anchors_x(1), map.anchors_y(1), 0, 'Xy', 'MarkerSize', 10); %leader's first position known
    plot3(map.anchors_x(1), map.anchors_y(1), map.anchors_z(1), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    for i = 2:length(map.anchors_x)
        % Generate random color
        color = [min(1,0.01*map.anchors_x(i)),max(0,1-0.01*map.anchors_x(i)),max(1-0.01*map.anchors_x(i))];
        % Plot anchors
        plot3(map.anchors_x(i), map.anchors_y(i), map.anchors_z(i), 'o', 'MarkerFaceColor', color, 'MarkerSize', 10);
        plot3(map.anchors_x(i), map.anchors_y(i), (map.anchors_z(i)+map.anchor_offset), 'Xy', 'MarkerSize', 10);
    end
end