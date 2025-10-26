%% Franka Emika Panda IK 비교 실험 통합 스크립트
% 
% 이 스크립트는 다음 세 가지 시나리오를 순차적으로 실행합니다.
% 1. 센서 노이즈 (관측 불확실성)
% 2. 로봇 모델 오차 (FK 불확실성)
% 3. 조인트 제어 오차 (제어 불확실성)
% 
% 각 시나리오는 독립적으로 실행되며, 최종적으로 모든 오차가 통합된
% 시나리오도 함께 평가합니다.

clc;
clear;
close all;

% --- Add 'src' and all its subfolders to the path ---

% Get the directory where this script (test.m) is running
scriptDir = fileparts(mfilename('fullpath')); 

% Get the full path to the 'src' directory
srcDir = fullfile(scriptDir, 'src');

% Add 'src' AND all its subdirectories (like optimizers, utils)
addpath(genpath(srcDir));

%% 공통 파라미터 설정
x_d_true = [0.4; 0.2; 0.3; 0; 0; 0]; % 실제 목표 위치 (Ground Truth)
panda = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
theta0_full = homeConfiguration(panda);
theta0 = theta0_full(1:7); % 7개 관절만 사용
num_trials = 20; % 반복 실험 횟수
max_iter = 1000; % Stochastic IK 반복 횟수

fprintf('프랑카 판다 로봇 IK 비교 실험을 시작합니다.\n');
fprintf('============================================\n');

%% 시나리오 0: Deterministic
fprintf('\n[시나리오 0: Deterministic] 실험 중...\n');

losses_stoch_s1 = zeros(num_trials, 1);
losses_builtin_s1 = zeros(num_trials, 1);

for i = 1:num_trials

    % IK 솔버 실행
    [theta_s, ~] = enhanced_localized_random_search(x_d_true, theta0, max_iter);
    [theta_b, ~] = franka_ik_builtin(x_d_true, theta0);

    % 실제 목표 위치(x_d_true)와의 오차 계산
    losses_stoch_s1(i) = ik_loss(theta_s, x_d_true);
    losses_builtin_s1(i) = ik_loss(theta_b(1:7), x_d_true);
end

fprintf('▶ 평균 Stochastic loss: %.5f\n', mean(losses_stoch_s1));
fprintf('▶ 평균 Built-in loss:   %.5f\n', mean(losses_builtin_s1));

%% 시나리오 1: 센서 노이즈 (관측 불확실성)
% 의미: "센서가 목표 위치를 약간 잘못 알려줌"
fprintf('\n[시나리오 1: 센서 노이즈] 실험 중...\n');

losses_stoch_s1 = zeros(num_trials, 1);
losses_builtin_s1 = zeros(num_trials, 1);

for i = 1:num_trials
    % 관측 노이즈 추가 (위치 ±5mm)
    obs_noise = 0.005 * randn(6,1);
    x_d_noisy = x_d_true + obs_noise;

    % IK 솔버 실행
    [theta_s, ~] = enhanced_localized_random_search(x_d_noisy, theta0, max_iter);
    [theta_b, ~] = franka_ik_builtin(x_d_noisy, theta0);

    % 실제 목표 위치(x_d_true)와의 오차 계산
    losses_stoch_s1(i) = ik_loss(theta_s, x_d_true);
    losses_builtin_s1(i) = ik_loss(theta_b(1:7), x_d_true);
end

fprintf('▶ 평균 Stochastic loss: %.5f\n', mean(losses_stoch_s1));
fprintf('▶ 평균 Built-in loss:   %.5f\n', mean(losses_builtin_s1));


%% 시나리오 2: 모델 불확실성 (FK 오차)
% 의미: "Forward kinematics 모델에 오차가 있음"
fprintf('\n[시나리오 2: 모델 불확실성] 실험 중...\n');

losses_stoch_s2 = zeros(num_trials, 1);
losses_builtin_s2 = zeros(num_trials, 1);

for i = 1:num_trials
    % IK 솔버는 정확한 FK 모델을 사용한다고 가정하고 실행
    [theta_s, ~] = enhanced_localized_random_search(x_d_true, theta0, max_iter);
    [theta_b, ~] = franka_ik_builtin(x_d_true, theta0);

    % 실제 로봇의 FK 연산 시 노이즈가 발생한다고 가정
    fk_noise = 0.005 * randn(6,1);
    x_s_actual = franka_forward_kinematics(theta_s) + fk_noise;
    x_b_actual = franka_forward_kinematics(theta_b(1:7)) + fk_noise;

    % 최종 위치와 실제 목표 위치(x_d_true)와의 오차 계산
    losses_stoch_s2(i) = norm(x_s_actual - x_d_true);
    losses_builtin_s2(i) = norm(x_b_actual - x_d_true);
end

fprintf('▶ 평균 Stochastic loss: %.6f\n', mean(losses_stoch_s2));
fprintf('▶ 평균 Built-in loss : %.6f\n', mean(losses_builtin_s2));


%% 시나리오 3: 제어 오차 (조인트 노이즈)
% 의미: "계산된 관절각 θ를 로봇이 정확히 따라가지 못함"
fprintf('\n[시나리오 3: 제어 오차] 실험 중...\n');

losses_stoch_s3 = zeros(num_trials, 1);
losses_builtin_s3 = zeros(num_trials, 1);

for i = 1:num_trials
    % IK 솔버 실행
    [theta_s, ~] = enhanced_localized_random_search(x_d_true, theta0, max_iter);
    [theta_b, ~] = franka_ik_builtin(x_d_true, theta0);

    % 계산된 관절각에 제어 오차(노이즈) 추가 (±1도)
    control_noise = deg2rad(1.0) * randn(7,1);
    theta_s_actual = theta_s + control_noise;
    theta_b_actual = theta_b(1:7) + control_noise;

    % 실제 로봇의 최종 위치 계산
    x_s_actual = franka_forward_kinematics(theta_s_actual);
    x_b_actual = franka_forward_kinematics(theta_b_actual);

    % 최종 위치와 실제 목표 위치(x_d_true)와의 오차 계산
    losses_stoch_s3(i) = norm(x_s_actual - x_d_true);
    losses_builtin_s3(i) = norm(x_b_actual - x_d_true);
end

fprintf('▶ 평균 Stochastic loss: %.6f\n', mean(losses_stoch_s3));
fprintf('▶ 평균 Built-in loss : %.6f\n', mean(losses_builtin_s3));
fprintf('============================================\n');


%% 결과 시각화
figure;
subplot(1,3,1);
bar([mean(losses_stoch_s1), mean(losses_builtin_s1)]);
set(gca, 'XTickLabel', {'Stochastic', 'Built-in'});
ylabel('평균 오차');
title('[시나리오 1] 센서 노이즈');
grid on;

subplot(1,3,2);
bar([mean(losses_stoch_s2), mean(losses_builtin_s2)]);
set(gca, 'XTickLabel', {'Stochastic', 'Built-in'});
title('[시나리오 2] 모델 불확실성');
grid on;

subplot(1,3,3);
bar([mean(losses_stoch_s3), mean(losses_builtin_s3)]);
set(gca, 'XTickLabel', {'Stochastic', 'Built-in'});
title('[시나리오 3] 제어 오차');
grid on;

sgtitle('IK 방법별 불확실성 시나리오 성능 비교');