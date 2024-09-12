function plotWaterFlow3d(xmin, xmax, ymin, ymax, zmin, zmax, vx, vy, vz)

    hold on;

    % Bottom face
    fill3([xmin, xmax, xmax, xmin], [ymin, ymin, ymax, ymax], [zmin, zmin, zmin, zmin], 'b', 'FaceAlpha', 0.3);
    % Top face
    fill3([xmin, xmax, xmax, xmin], [ymin, ymin, ymax, ymax], [zmax, zmax, zmax, zmax], 'b', 'FaceAlpha', 0.3);
    % Side faces
    fill3([xmax, xmin, xmin, xmax], [ymin, ymin, ymin, ymin], [zmin, zmin, zmax, zmax], 'b', 'FaceAlpha', 0.3);
    fill3([xmax, xmin, xmin, xmax], [ymax, ymax, ymax, ymax], [zmin, zmin, zmax, zmax], 'b', 'FaceAlpha', 0.3);
    fill3([xmin, xmin, xmin, xmin], [ymin, ymax, ymax, ymin], [zmin, zmin, zmax, zmax], 'b', 'FaceAlpha', 0.3);
    fill3([xmax, xmax, xmax, xmax], [ymin, ymax, ymax, ymin], [zmin, zmin, zmax, zmax], 'b', 'FaceAlpha', 0.3);

    % Draw arrow to indicate water flow intensity and direction
    flow_center_x = (xmin + xmax) / 2;
    flow_center_y = (ymin + ymax) / 2;
    flow_center_z = (zmin + zmax) / 2;
    drawArrow3D([flow_center_x-4*vx,flow_center_y-4*vy,flow_center_z-4*vz],[flow_center_x+4*vx,flow_center_y+4*vy,flow_center_z+4*vz],'b',3);

end

