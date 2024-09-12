function [next] = speedLimit3D(current, next, dt, max_vel, max_acc)
    % current = [v_x v_y v_z];
    % Kinematic = [max_vel_x; max_vel_y; max_vel_z; max_acc_x; max_acc_y; max_acc_z];
    
    Kinematic = [max_vel; max_vel; max_vel; max_acc; max_acc; max_acc];  % Define maximum velocities and accelerations
    
    % Limitation on speed x
    delta_x = next(1) - current(1);
    if delta_x >= 0
        next(1) = min(current(1) + delta_x, current(1) + Kinematic(4) * dt);
    else
        next(1) = max(current(1) + delta_x, current(1) - Kinematic(4) * dt);
    end
    if next(1) >= 0
        next(1) = min(next(1), Kinematic(1));
    else
        next(1) = max(next(1), -Kinematic(1));
    end
    
    % Limitation on speed y
    delta_y = next(2) - current(2);
    if delta_y >= 0
        next(2) = min(current(2) + delta_y, current(2) + Kinematic(5) * dt);
    else
        next(2) = max(current(2) + delta_y, current(2) - Kinematic(5) * dt);
    end
    if next(2) >= 0
        next(2) = min(next(2), Kinematic(2));
    else
        next(2) = max(next(2), -Kinematic(2));
    end
    
    % Limitation on speed z
    delta_z = next(3) - current(3);
    if delta_z >= 0
        next(3) = min(current(3) + delta_z, current(3) + Kinematic(6) * dt);
    else
        next(3) = max(current(3) + delta_z, current(3) - Kinematic(6) * dt);
    end
    if next(3) >= 0
        next(3) = min(next(3), Kinematic(3));
    else
        next(3) = max(next(3), -Kinematic(3));
    end
end
