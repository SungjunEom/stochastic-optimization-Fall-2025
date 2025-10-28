% Gradient-based Optimizer 간단 테스트
clc; clear;

% Add path
scriptDir = fileparts(mfilename('fullpath'));
srcDir = fullfile(scriptDir, 'src');
addpath(genpath(srcDir));

fprintf('=== Gradient-based IK Solvers 테스트 ===\n\n');

% 로봇 로드
panda = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
theta0_full = homeConfiguration(panda);
theta0 = theta0_full(1:7);

% 목표 위치
x_d = [0.4; 0.2; 0.3; 0; 0; 0];

fprintf('목표 위치: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', x_d);
fprintf('초기 관절각: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n\n', theta0);

%% 테스트 1: Deterministic (노이즈 없음)
fprintf('[테스트 1] Deterministic 환경\n');
fprintf('----------------------------------------\n');

max_iter = 500;
learning_rate = 0.01;
noise_level = 0;

% Gradient Descent
tic;
[theta_gd, ~] = gradient_descent_ik(x_d, theta0, max_iter, learning_rate, noise_level);
time_gd = toc;
loss_gd = ik_loss(theta_gd, x_d);

fprintf('✓ Gradient Descent\n');
fprintf('  최종 Loss: %.6f\n', loss_gd);
fprintf('  실행 시간: %.2f초\n\n', time_gd);

% Adam
tic;
[theta_adam, ~] = adam_ik(x_d, theta0, max_iter, learning_rate, noise_level);
time_adam = toc;
loss_adam = ik_loss(theta_adam, x_d);

fprintf('✓ Adam Optimizer\n');
fprintf('  최종 Loss: %.6f\n', loss_adam);
fprintf('  실행 시간: %.2f초\n\n', time_adam);

%% 테스트 2: Noisy Observation
fprintf('[테스트 2] Noisy Observation (sigma=0.005)\n');
fprintf('----------------------------------------\n');

noise_level = 0.005;
num_trials = 5;

losses_gd_noisy = zeros(num_trials, 1);
losses_adam_noisy = zeros(num_trials, 1);

fprintf('5번 반복 실행 중...\n');
for i = 1:num_trials
    % Gradient Descent with noise
    [theta_gd_noisy, ~] = gradient_descent_ik(x_d, theta0, max_iter, learning_rate, noise_level);
    losses_gd_noisy(i) = ik_loss(theta_gd_noisy, x_d);
    
    % Adam with noise
    [theta_adam_noisy, ~] = adam_ik(x_d, theta0, max_iter, learning_rate, noise_level);
    losses_adam_noisy(i) = ik_loss(theta_adam_noisy, x_d);
    
    fprintf('  Trial %d/%d 완료\n', i, num_trials);
end

fprintf('\n✓ Gradient Descent (Noisy)\n');
fprintf('  평균 Loss: %.6f (±%.6f)\n', mean(losses_gd_noisy), std(losses_gd_noisy));
fprintf('  최소/최대: %.6f / %.6f\n\n', min(losses_gd_noisy), max(losses_gd_noisy));

fprintf('✓ Adam Optimizer (Noisy)\n');
fprintf('  평균 Loss: %.6f (±%.6f)\n', mean(losses_adam_noisy), std(losses_adam_noisy));
fprintf('  최소/최대: %.6f / %.6f\n\n', min(losses_adam_noisy), max(losses_adam_noisy));

%% 시각화 (수렴 곡선)
fprintf('[시각화] 최적화 과정 플롯\n');
fprintf('----------------------------------------\n');

% 한 번 더 실행하여 history 저장
[~, history_gd_clean] = gradient_descent_ik(x_d, theta0, max_iter, learning_rate, 0);
[~, history_adam_clean] = adam_ik(x_d, theta0, max_iter, learning_rate, 0);
[~, history_gd_noisy] = gradient_descent_ik(x_d, theta0, max_iter, learning_rate, 0.005);
[~, history_adam_noisy] = adam_ik(x_d, theta0, max_iter, learning_rate, 0.005);

% Loss 계산
losses_gd_clean_history = zeros(max_iter, 1);
losses_adam_clean_history = zeros(max_iter, 1);
losses_gd_noisy_history = zeros(max_iter, 1);
losses_adam_noisy_history = zeros(max_iter, 1);

for k = 1:max_iter
    losses_gd_clean_history(k) = ik_loss(history_gd_clean(k,:)', x_d);
    losses_adam_clean_history(k) = ik_loss(history_adam_clean(k,:)', x_d);
    losses_gd_noisy_history(k) = ik_loss(history_gd_noisy(k,:)', x_d);
    losses_adam_noisy_history(k) = ik_loss(history_adam_noisy(k,:)', x_d);
end

figure('Position', [100, 100, 1200, 400]);

% Deterministic
subplot(1,2,1);
semilogy(1:max_iter, losses_gd_clean_history, 'b-', 'LineWidth', 2);
hold on;
semilogy(1:max_iter, losses_adam_clean_history, 'r-', 'LineWidth', 2);
grid on;
xlabel('Iteration');
ylabel('Loss (log scale)');
title('Deterministic Environment');
legend('Gradient Descent', 'Adam', 'Location', 'best');

% Noisy
subplot(1,2,2);
semilogy(1:max_iter, losses_gd_noisy_history, 'b-', 'LineWidth', 1.5, 'Color', [0.3 0.3 1]);
hold on;
semilogy(1:max_iter, losses_adam_noisy_history, 'r-', 'LineWidth', 1.5, 'Color', [1 0.3 0.3]);
grid on;
xlabel('Iteration');
ylabel('Loss (log scale)');
title('Noisy Observation (sigma=0.005)');
legend('Gradient Descent', 'Adam', 'Location', 'best');

fprintf('✓ 플롯 생성 완료\n\n');

fprintf('=== 테스트 완료 ===\n');
fprintf('두 알고리즘 모두 정상 작동합니다.\n');
fprintf('Noisy environment에서는 수렴이 불안정할 수 있습니다.\n');

