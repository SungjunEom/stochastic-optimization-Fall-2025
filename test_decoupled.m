%% simul 1
% 1. 노이즈를 x_d에 넣는 경우 — 관측 기반 IK

% 의미: "센서가 목표 위치를 약간 잘못 알려줌"
% 
% 시나리오:
% % 사람이 원하는 목표 위치를 센서가 인식함 (예: 비전 시스템, 포인트클라우드)
% % 위치 센서 노이즈, 카메라 캘리브레이션 오차 등
% 
% 적용:
% % x_d_noisy = x_d + ε
% % IK는 이 x_d_noisy를 만족하는 θ를 찾음
% % 장점: 노이즈가 IK 입력에 직접 영향을 주는 현실적 상황 시뮬레이션 가능

x_d = [0.4; 0.2; 0.3; 0; 0; 0];

num_trials = 20;
losses_stoch = zeros(num_trials, 1);
losses_builtin = zeros(num_trials, 1);

for i = 1:num_trials
    noise = 0.005 * randn(6,1);  % 위치 ±5mm, 자세 ±약간
    x_d_noisy = x_d + noise;

    [theta_s, ~] = franka_ik_stochastic(x_d_noisy, theta0, 1000);
    [theta_b, ~] = franka_ik_builtin(x_d_noisy, theta0);

    losses_stoch(i) = ik_loss(theta_s, x_d);              % 실제 목표와의 차이
    losses_builtin(i) = ik_loss(theta_b(1:7), x_d);
end

fprintf('📊 평균 Stochastic loss: %.5f\n', mean(losses_stoch));
fprintf('📊 평균 Built-in loss:   %.5f\n', mean(losses_builtin));



%% simul 2
% 2. 노이즈를 x(θ)에 넣는 경우 — 모델 기반 불확실성
% 
% 의미: "Forward kinematics 모델에 오차가 있음"
% 
% 시나리오:
% % 로봇 링크 길이 부정확, 조인트 백래시 존재
% % 실제 로봇 동작 결과 x(θ)에 잡음이 섞임
% 
% 적용:
% % x_actual = f(θ) + ε, 그래서 loss 계산 시 오차 존재
clc; clear;


x_d_true = [0.4; 0.2; 0.3; 0; 0; 0];
theta0 = zeros(7,1);
max_iter = 5000;
num_trials = 20;
losses_stoch = zeros(num_trials, 1);
losses_builtin = zeros(num_trials, 1);


for i = 1:num_trials
x_d_noisy = x_d_true + 0.02 * randn(6,1); % 관측값 오염


[theta_s, ~] = franka_ik_stochastic(x_d_noisy, theta0, max_iter);
[theta_b, ~] = franka_ik_builtin(x_d_noisy, theta0);


x_s = franka_forward_kinematics(theta_s) + 0.005*randn(6,1); % FK 출력 노이즈
x_b = franka_forward_kinematics(theta_b(1:7)) + 0.005*randn(6,1);


losses_stoch(i) = norm(x_s - x_d_true);
losses_builtin(i) = norm(x_b - x_d_true);
end


fprintf('\n[시나리오 2 - 모델 불확실성] 평균 오차\n');
fprintf('Stochastic IK: %.6f\n', mean(losses_stoch));
fprintf('Built-in IK : %.6f\n', mean(losses_builtin));


figure;
bar([mean(losses_stoch), mean(losses_builtin)]);
set(gca, 'XTickLabel', {'Stochastic', 'Built-in'});
ylabel('평균 Position Error (m)');
title('시나리오 2 - 모델 불확실성(FK 노이즈)');
%% simul 3
% 3. 노이즈를 θ에 넣는 경우 — 조인트 제어 오차
% 
% 의미: "계산된 θ를 정확히 따라가지 못함"
% 
% 시나리오:
% % 모터 정밀도 부족, 관절 backlash 등으로 오차 발생
% 
% 적용:
% % θ_actual = θ_cmd + ε
% % 실험에서는 θ → θ + noise 한 후 FK

clc;


x_d_true = [0.4; 0.2; 0.3; 0; 0; 0];
theta0 = zeros(7,1);
max_iter = 5000;
num_trials = 20;
losses_stoch = zeros(num_trials, 1);
losses_builtin = zeros(num_trials, 1);


for i = 1:num_trials
x_d_noisy = x_d_true + 0.02 * randn(6,1); % 센서 관측값 오염


[theta_s, ~] = franka_ik_stochastic(x_d_noisy, theta0, max_iter);
[theta_b, ~] = franka_ik_builtin(x_d_noisy, theta0);


control_noise = deg2rad(1.0) * randn(7,1); % 조인트 오차 ±1도
theta_s_actual = theta_s + control_noise;
theta_b_actual = theta_b(1:7) + control_noise;


x_s = franka_forward_kinematics(theta_s_actual);
x_b = franka_forward_kinematics(theta_b_actual);


losses_stoch(i) = norm(x_s - x_d_true);
losses_builtin(i) = norm(x_b - x_d_true);
end


fprintf('\n[시나리오 3 - 제어 오차] 평균 오차\n');
fprintf('Stochastic IK: %.6f\n', mean(losses_stoch));
fprintf('Built-in IK : %.6f\n', mean(losses_builtin));


figure;
bar([mean(losses_stoch), mean(losses_builtin)]);
set(gca, 'XTickLabel', {'Stochastic', 'Built-in'});
ylabel('평균 Position Error (m)');
title('시나리오 3 - 제어기 오차 (조인트 노이즈)');
