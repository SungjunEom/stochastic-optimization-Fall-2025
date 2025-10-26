%% Robot Motion Planning

% Initialization
clear;
close all;

%% Load the robot model
robot = loadrobot("frankaEmikaPanda", "DataFormat", "row"); % use frankaEmikaPanda robot
numJoints = numel(robot.homeConfiguration);

%% Set the mode
mode = 0; % 0: Input acceleration/velocity, 1: Input motion time

%% Set the simulation time
simulation_time = 4;
dt = 0.01;
n_knots = 8; % number of knots

pos_x = zeros(n_knots, numJoints);
pos_x(1, :) = [0 0 0 0 0 0 0 0 0];
pos_x(2, :) = [10 0 0 10 0 20 10 0 0];
pos_x(3, :) = [0 -10 0 10 20 10 20 0 0];
pos_x(4, :) = [0 0 -30 50 50 20 30 0 0];
pos_x(5, :) = [0 20 0 50 10 20 20 0 0];
pos_x(6, :) = [30 10 0 70 20 20 10 0 0];
pos_x(7, :) = [-10 -20 30 90 -10 -20 -5 0 0];
pos_x(8, :) = [0 0 0 0 0 0 0 0 0];
pos_x = pos_x * pi / 180; % deg to rad
v = zeros(n_knots, numJoints); % v0 = 0, v1 = 0
h = ones(1, n_knots - 1) * simulation_time / (n_knots - 1); % time_interval : uniform time intervals

%% Cubic Spline Interpolation for each joint

% make A matrix refer to time intervals
A = zeros(n_knots - 2, n_knots - 2);
% make vector b for each joint
b = zeros(n_knots - 2, numJoints);

for i = 1:n_knots - 2
    for j = 1:n_knots - 2
        if j == i
            A(i, j) = 2 * (h(i) + h(i + 1));
        elseif j == i + 1
            A(i, j) = h(i);
        elseif j == i - 1
            A(i, j) = h(i + 1);
        else
            A(i, j) = 0;
        end
    end

    b(i, :) = 3 / (h(i) * h(i + 1)) * (h(i) * h(i) * (pos_x(i + 2, :) - pos_x(i + 1, :)) + h(i + 1) * h(i + 1) * (pos_x(i + 1, :) - pos_x(i, :)));
    if i == 1
        b(i, :) = b(i, :) - h(i + 1) * v(i, :);
    elseif i == n_knots - 2
        b(i, :) = b(i, :) - h(i) * v(i + 2, :);
    end
end

% make velocity for each knot except start and end
v(2:n_knots - 1, :) = A \ b(:, :);

% make cubic equation coefficients for each section and each joint
a = zeros(n_knots - 1, numJoints, 4); % a(cubic, joint, coeff)

for k = 1:n_knots - 1
    a(k, :, 1) = pos_x(k, :); % a(k, 1, j) = p0
    a(k, :, 2) = v(k, :); % a(k, 2, j) = v0
    a(k, :, 3) = (3 * (pos_x(k + 1, :) - pos_x(k, :)) - 2 * h(k) * v(k, :) - h(k) * v(k + 1, :)) / h(k)^2;
    a(k, :, 4) = (-2 * (pos_x(k + 1, :) - pos_x(k, :)) + h(k) * (v(k, :) + v(k + 1, :))) / h(k)^3;
end

function [index, tau] = getTimeIndex(t, h, n_knots)
    cum_h = cumsum(h);
    for idx = 1:n_knots - 1
        if t <= cum_h(idx)
            index = idx;
            if idx == 1
                tau = t;
            else
                tau = t - cum_h(idx - 1);
            end
            return;
        end
    end
    index = n_knots - 1; % if t > total time, return last index
    tau = t - cum_h(index - 1);
end

%% Simulate
pos_array = zeros(round(simulation_time / dt) + 1, numJoints);
vel_array = zeros(round(simulation_time / dt) + 1, numJoints);
acc_array = zeros(round(simulation_time / dt) + 1, numJoints);
t_array = zeros(round(simulation_time / dt) + 1, 1);

for t = 0:dt:simulation_time
    [idx, tau] = getTimeIndex(t, h, n_knots);
    pos_array(round(t / dt) + 1, :) = a(idx, :, 1) + a(idx, :, 2) * tau + a(idx, :, 3) * tau^2 + a(idx, :, 4) * tau^3;
    vel_array(round(t / dt) + 1, :) = a(idx, :, 2) + 2 * a(idx, :, 3) * tau + 3 * a(idx, :, 4) * tau^2;
    acc_array(round(t / dt) + 1, :) = 2 * a(idx, :, 3) + 6 * a(idx, :, 4) * tau;
    t_array(round(t / dt) + 1) = t;
end

%% Visualize the simulation
% plot joint angles, velocities, accelerations
y_label = ["joint angle(rad)", "joint velocity(rad/s)", "joint acceleration(rad/s^2)"];
figure(1);
data_array = {pos_array, vel_array, acc_array};
for d = 1:3
    subplot(2, 2, d)
    hold on;
    grid on;
    for i = 1:numJoints
        plot(t_array, data_array{d}(:, i));
    end
    xlabel('time (s)');
    ylabel(y_label(d));
    title([y_label(d)]);
    legend('joint 1', 'joint 2', 'joint 3', 'joint 4', 'joint 5', 'joint 6', 'joint 7');
end

subplot(2, 2, 4)
set(gcf, 'Name', 'Motion Simulation', 'NumberTitle', 'off');

% initial pose
show(robot, pos_array(1, :));
view(145, 25); % 보는 각도 조절 [수평, 수직]
axis([-1 1 -1 1 -0.2 1.5]); % 축 범위 설정 [xmin xmax ymin ymax zmin zmax]
hold on;
title(['Simulation Time: ', num2str(t_array(1), '%.2f'), 's']);

% update poses
for i = 1:length(t_array)
    show(robot, pos_array(i, :), 'PreservePlot', false);
    title(['Simulation Time: ', num2str(t_array(i), '%.2f'), 's']);
    drawnow;
end

hold off;
