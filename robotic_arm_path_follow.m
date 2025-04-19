function robotic_arm_path2_follow(points)
    L1 = 10;
    L2 = 10;
    steps_per_segment = 50;
    max_reach = L1 + L2;
    valid_indices = sqrt(points(:,1).^2 + points(:,2).^2) <= max_reach;
    points = points(valid_indices, :);
    if size(points, 1) < 2
        error('Not enough valid points to form a path');
    end
    figure;
    hold on;
    axis equal;
    grid on;
    title('2-Link Robotic Arm Path Following');
    xlabel('X');
    ylabel('Y');
    plot(points(:,1), points(:,2), 'ko--', 'MarkerFaceColor', 'k');
    for i = 1:size(points, 1) - 1
        x_segment = linspace(points(i,1), points(i+1,1), steps_per_segment);
        y_segment = linspace(points(i,2), points(i+1,2), steps_per_segment);
        for j = 1:steps_per_segment
            x = x_segment(j);
            y = y_segment(j);
            r = sqrt(x^2 + y^2);
            if r > max_reach || r < abs(L1-L2)
                warning('Point (%f, %f) is not reachable', x, y);
                continue;
            end
            cos_theta2 = (r^2 - L1^2 - L2^2) / (2 * L1 * L2);
            if abs(cos_theta2) > 1
                warning('Point (%f, %f) is not reachable (invalid cos_theta2)', x, y);
                continue;
            end
            theta2 = atan2(-sqrt(1 - cos_theta2^2), cos_theta2);
            beta = atan2(y, x);
            gamma = atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
            theta1 = beta - gamma;
            x1 = L1 * cos(theta1);
            y1 = L1 * sin(theta1);
            x2 = x1 + L2 * cos(theta1 + theta2);
            y2 = y1 + L2 * sin(theta1 + theta2);
            cla;
            plot(points(:,1), points(:,2), 'ko--', 'MarkerFaceColor', 'k');
            plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
            plot([0, x1], [0, y1], 'b', 'LineWidth', 6);
            plot([x1, x2], [y1, y2], 'r', 'LineWidth', 8);
            plot(x1, y1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
            plot(x2, y2, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
            plot(x, y, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
            xlim([-20, 20]);
            ylim([-20, 25]);
            text(-18, 20, sprintf('θ1: %.1f°, θ2: %.1f°', theta1*180/pi, theta2*180/pi));
            err = sqrt((x2-x)^2 + (y2-y)^2);
            text(-18, 18, sprintf('Error: %.2f units', err));
            pause(0.01);
        end
    end
end
