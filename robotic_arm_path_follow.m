function robotic_arm_path_follow(points)
    L1 = 10;
    L2 = 10;
    steps_per_segment = 50;
    max_reach = L1 + L2;

    % Filter points within reach
    points = points(sqrt(points(:,1).^2 + points(:,2).^2) <= max_reach, :);

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

            if (x^2 + y^2) > max_reach^2
                continue
            end

            D = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
            if abs(D) > 1
                continue
            end

            theta2 = atan2(sqrt(1 - D^2), D);  % Elbow angle
            theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));  % Shoulder angle

            % Joint positions
            x1 = L1 * cos(theta1);
            y1 = L1 * sin(theta1);
            x2 = x1 + L2 * cos(theta1 + theta2);
            y2 = y1 + L2 * sin(theta1 + theta2);

            % Clear previous frame
            cla;
            plot(points(:,1), points(:,2), 'ko--', 'MarkerFaceColor', 'k');
            plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');  % base
            plot([0, x1], [0, y1], 'b', 'LineWidth', 8);       % Link 1
            plot([x1, x2], [y1, y2], 'r', 'LineWidth', 8);     % Link 2
            plot(x1, y1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');  % elbow
            plot(x2, y2, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');  % end effector

            xlim([-20, 20]);
            ylim([-20, 25]);

            pause(0.01);
        end
    end
end

