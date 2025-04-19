function robotic_arm_parabola(a, b, c)
    L1 = 10;
    L2 = 10;
    x_vals = linspace(-10, 10, 200);
    y_vals = a * x_vals.^2 + b * x_vals + c;
    figure;
    hold on;
    plot(x_vals, y_vals, 'k--', 'LineWidth', 1.5);
    axis equal;
    grid on;
    title('2-Link Robotic Arm Following a Parabola');
    xlabel('X');
    ylabel('Y');
    for i = 1:length(x_vals)
        x = x_vals(i);
        y = y_vals(i);
        D = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
        if abs(D) > 1
            continue
        end
        theta2 = atan2(sqrt(1 - D^2), D);
        theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
        x1 = L1 * cos(theta1);
        y1 = L1 * sin(theta1);
        x2 = x1 + L2 * cos(theta1 + theta2);
        y2 = y1 + L2 * sin(theta1 + theta2);
        cla;
        plot(x_vals, y_vals, 'k--', 'MarkerFaceColor', 'k');
        plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
        plot([0, x1], [0, y1], 'b', 'LineWidth', 6);
        plot([x1, x2], [y1, y2], 'r', 'LineWidth', 8);
        plot(x1, y1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        plot(x2, y2, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        plot(x, y, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
        xlim([-20, 20]);
        ylim([-25, 25]);
        pause(0.02);
    end
end
