% function repulsion = computeRepulsion3D(robot_pose, obs_pose, detect_R)
%     % compute the repulsion using artificial potential field method
%     % obs_pose=[x1 y1 z1; x2 y2 z2; ....]
%     [M, ~] = size(obs_pose);
%     repulsion = zeros(1, 3); % repulsion vector in x, y, z directions
%     for i = 1:M
%         distance = norm(robot_pose - obs_pose(i, :));
%         if distance <= detect_R
%             % Calculate the repulsion force with saturation and adjusted distance term
%             temp = min((1 / distance) * 3*exp(-distance / (4*detect_R)), 8) / distance; % 3*exp, 4*detectR, 8: tuning constants
%             repulsion = repulsion + temp * (robot_pose - obs_pose(i, :));
%         end
%     end
% end
%% vectorized form
function repulsion = computeRepulsion3D(robot_pose, obs_pose, detect_R)
    % Compute the repulsion using artificial potential field method
    % obs_pose=[x1 y1 z1; x2 y2 z2; ....]
    
    % Compute the distances from the robot to each obstacle
    distances = vecnorm(obs_pose - robot_pose, 2, 2);
    
    % Identify obstacles within the detection radius
    within_radius = distances <= detect_R;
    
    % Compute repulsion forces only for obstacles within the detection radius
    valid_obs_pose = obs_pose(within_radius, :);
    valid_distances = distances(within_radius);
    
    % Calculate the repulsion force with saturation and adjusted distance term
    temp = min((1 ./ valid_distances) .* 3 .* exp(-valid_distances / (4 * detect_R)), 8) ./ valid_distances;
    
    % Compute the repulsion vector
    repulsion_vectors = temp .* (robot_pose - valid_obs_pose);
    repulsion = sum(repulsion_vectors, 1);
end

