% 테스트 스크립트: 시나리오 4 - 모든 불확실성 동시 적용
clc; clear;

%% 로봇 모델 로드 (Franka Emika Panda)
try
    panda = loadrobot("frankaEmikaPanda", "DataFormat", "column", "Gravity", [0 0 -9.81]);
catch
    error("Robotics System Toolbox가 설치되어 있어야 합니다. loadrobot 사용 불가");
end

%% 목표 자세 설정 (랜덤 또는 고정)
use_random_target = true;  % <- 여기서 true/false로 선택

if use_random_target
    % 위치: 0.3~0.5, 자세: -pi~pi 범위
    x_d_true = [0.3 + 0.2*rand(3,1); -pi + 2*pi*rand(3,1)];
else
    x_d_true = [0.4; 0.2; 0.3; 0; 0; 0];
end

theta0 = zeros(7,1);
max_iter = 1000;
num_trials = 20;

losses_stoch = zeros(num_trials, 1);
losses_builtin = zeros(num_trials, 1);

for i = 1:num_trials
    %% 센서 노이즈
    x_d_noisy = x_d_true + 0.02 * randn(6,1);

    %% IK 수행
    [theta_s, ~] = franka_ik_stochastic(x_d_noisy, theta0, max_iter);
    [theta_b, ~] = franka_ik_builtin(x_d_noisy, theta0);

    %% 제어기 조인트 오차 적용
    joint_noise = deg2rad(1.0) * randn(7,1);
    theta_s_actual = theta_s + joint_noise;
    theta_b_actual = theta_b(1:7) + joint_noise;

    % 9x1로 확장 (gripper joint 2개는 0으로 채움)
    theta_s_9 = [theta_s_actual; 0; 0];
    theta_b_9 = [theta_b_actual; 0; 0];

    %% FK 계산 후 모델 불확실성 노이즈 추가
    x_s = franka_forward_kinematics(theta_s_actual) + 0.005 * randn(6,1);
    x_b = franka_forward_kinematics(theta_b_actual) + 0.005 * randn(6,1);

    %% 진짜 목표 기준 오차 측정
    losses_stoch(i) = norm(x_s - x_d_true);
    losses_builtin(i) = norm(x_b - x_d_true);
   

    %% 로봇 자세 시각화 (마지막 반복만 출력)
    if i == num_trials
               
        figure;
        show(panda, theta_s_9, 'PreservePlot', false, 'Frames','off');
        title('📌 Stochastic IK 최종 자세');
        hold on;
        show(panda, theta_b_9, 'PreservePlot', false, 'Frames','off');
        title('📌 Built-in IK 최종 자세');
        legend('Stochastic IK','Built-in IK');
        hold off;
    end
end

%% 결과 출력
fprintf('\n[시나리오 4 - 모든 불확실성 포함] 평균 오차\n');
fprintf('Stochastic IK: %.6f\n', mean(losses_stoch));
fprintf('Built-in IK  : %.6f\n', mean(losses_builtin));

%% 시각화
figure;
bar([mean(losses_stoch), mean(losses_builtin)]);
set(gca, 'XTickLabel', {'Stochastic', 'Built-in'});
ylabel('평균 Position Error (m)');
title('시나리오 4 - 모든 불확실성 동시 적용');