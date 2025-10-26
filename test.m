%% Franka Emika Panda IK 비교 실험 통합 스크립트
%
% 이 스크립트는 4개의 알고리즘을 4개의 시나리오에 대해 비교합니다.
%
% [알고리즘]
% 1. Built-in (Baseline)
% 2. Simple Random Search (SRS)
% 3. Localized Random Search (LRS)
% 4. Enhanced Localized Random Search (ELRS)
%
% [시나리오]
% 0. Deterministic (노이즈 없음)
% 1. 센서 노이즈 (관측 불확실성)
% 2. 로봇 모델 오차 (FK 불확실성)
% 3. 조인트 제어 오차 (제어 불확실성)
clc;
clear;
close all;
% --- Add 'src' and all its subfolders to the path ---
scriptDir = fileparts(mfilename('fullpath'));
srcDir = fullfile(scriptDir, 'src');
addpath(genpath(srcDir));
%% 공통 파라미터 설정
x_d_true = [0.4; 0.2; 0.3; 0; 0; 0]; % 실제 목표 위치 (Ground Truth)
panda = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
theta0_full = homeConfiguration(panda);
theta0 = theta0_full(1:7); % 7개 관절만 사용
num_trials = 20; % 반복 실험 횟수
max_iter = 1000; % Stochastic IK 반복 횟수
% --- 알고리즘 정의 ---
% 함수 핸들을 셀 배열로 정의
alg_handles = {
    @franka_ik_builtin, ...
    @simple_random_search, ...
    @localized_random_search, ...
    @enhanced_localized_random_search
};
% 그래프 및 출력용 이름
alg_names = {'Built-in', 'SRS', 'LRS', 'ELRS'};
num_algs = length(alg_names);
num_scenarios = 4; % 시나리오 개수
fprintf('프랑카 판다 로봇 IK 비교 실험을 시작합니다.\n');
fprintf('알고리즘: %s\n', strjoin(alg_names, ', '));
fprintf('============================================\n');
%% 시나리오 0: Deterministic
fprintf('\n[시나리오 0: Deterministic] 실험 중...\n');
losses_s0 = zeros(num_trials, num_algs);
for i = 1:num_trials
    for j = 1:num_algs
        solver_func = alg_handles{j};
        
        if j == 1 % Built-in solver (Baseline)
            [theta_sol, ~] = solver_func(x_d_true, theta0);
            theta_j = theta_sol(1:7);
        else % Stochastic solvers
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter);
        end
        
        % 실제 목표 위치(x_d_true)와의 오차 계산
        losses_s0(i, j) = ik_loss(theta_j, x_d_true);
    end
end
mean_losses_s0 = mean(losses_s0, 1);
for j = 1:num_algs
    fprintf('▶ %-10s mean loss: %.6f\n', [alg_names{j} ':'], mean_losses_s0(j));
end
%% 시나리오 1: 센서 노이즈 (관측 불확실성)
% 의미: "센서가 목표 위치를 약간 잘못 알려줌"
fprintf('\n[시나리오 1: 센서 노이즈] 실험 중...\n');
losses_s1 = zeros(num_trials, num_algs);
for i = 1:num_trials
    % 관측 노이즈 추가 (위치 ±5mm)
    obs_noise = 0.005 * randn(6,1);
    x_d_noisy = x_d_true + obs_noise;
    for j = 1:num_algs
        solver_func = alg_handles{j};
        
        if j == 1 % Built-in solver
            [theta_sol, ~] = solver_func(x_d_noisy, theta0);
            theta_j = theta_sol(1:7);
        else % Stochastic solvers
            [theta_j, ~] = solver_func(x_d_noisy, theta0, max_iter);
        end
        % 실제 목표 위치(x_d_true)와의 오차 계산
        losses_s1(i, j) = ik_loss(theta_j, x_d_true);
    end
end
mean_losses_s1 = mean(losses_s1, 1);
for j = 1:num_algs
    fprintf('▶ %-10s mean loss: %.6f\n', [alg_names{j} ':'], mean_losses_s1(j));
end
%% 시나리오 2: 모델 불확실성 (FK 오차)
% 의미: "Forward kinematics 모델에 오차가 있음"
fprintf('\n[시나리오 2: 모델 불확실성] 실험 중...\n');
losses_s2 = zeros(num_trials, num_algs);
for i = 1:num_trials
    
    theta_solutions = cell(num_algs, 1);
    
    % 1. 모든 솔버로 θ 계산 (정확한 x_d 사용)
    for j = 1:num_algs
        solver_func = alg_handles{j};
        if j == 1
            [theta_sol, ~] = solver_func(x_d_true, theta0);
            theta_solutions{j} = theta_sol(1:7);
        else
            [theta_solutions{j}, ~] = solver_func(x_d_true, theta0, max_iter);
        end
    end
    % 2. 동일한 FK 노이즈를 적용하여 실제 위치 계산 및 오차 평가
    fk_noise = 0.005 * randn(6,1);
    for j = 1:num_algs
        x_actual = franka_forward_kinematics(theta_solutions{j}) + fk_noise;
        % 최종 위치와 실제 목표 위치(x_d_true)와의 오차 계산
        losses_s2(i, j) = norm(x_actual - x_d_true);
    end
end
mean_losses_s2 = mean(losses_s2, 1);
for j = 1:num_algs
    fprintf('▶ %-10s mean loss: %.6f\n', [alg_names{j} ':'], mean_losses_s2(j));
end
%% 시나리오 3: 제어 오차 (조인트 노이즈)
% 의미: "계산된 관절각 θ를 로봇이 정확히 따라가지 못함"
fprintf('\n[시나리오 3: 제어 오차] 실험 중...\n');
losses_s3 = zeros(num_trials, num_algs);
for i = 1:num_trials
    
    theta_solutions = cell(num_algs, 1);
    
    % 1. 모든 솔버로 θ 계산 (정확한 x_d 사용)
    for j = 1:num_algs
        solver_func = alg_handles{j};
        if j == 1
            [theta_sol, ~] = solver_func(x_d_true, theta0);
            theta_solutions{j} = theta_sol(1:7);
        else
            [theta_solutions{j}, ~] = solver_func(x_d_true, theta0, max_iter);
        end
    end
    % 2. 동일한 제어 노이즈를 적용하여 실제 위치 계산 및 오차 평가
    control_noise = deg2rad(1.0) * randn(7,1);
    for j = 1:num_algs
        theta_actual = theta_solutions{j} + control_noise;
        x_actual = franka_forward_kinematics(theta_actual);
        
        % 최종 위치와 실제 목표 위치(x_d_true)와의 오차 계산
        losses_s3(i, j) = norm(x_actual - x_d_true);
    end
end
mean_losses_s3 = mean(losses_s3, 1);
for j = 1:num_algs
    fprintf('▶ %-10s mean loss: %.6f\n', [alg_names{j} ':'], mean_losses_s3(j));
end
fprintf('============================================\n');

% -----------------------------------------------------------------
% ----- [NEW] 통계 계산 (평균, 표준오차, 95% 신뢰구간) -----
% -----------------------------------------------------------------
fprintf('\n[통계 계산] 95%% 신뢰구간 계산 중...\n');
% --- 결과 취합 ---
all_losses = {losses_s0, losses_s1, losses_s2, losses_s3};
all_mean_losses = [mean_losses_s0; mean_losses_s1; mean_losses_s2; mean_losses_s3];
% --- 95% 신뢰구간 (CI) 계산 ---
% CI = mean ± t_value * (std / sqrt(n))
n = num_trials;
t_value = tinv(0.975, n - 1); % 95% CI (양측)에 대한 t-값
all_std_dev = zeros(num_scenarios, num_algs);
all_sem = zeros(num_scenarios, num_algs); % Standard Error of the Mean (표준 오차)
all_ci_half_width = zeros(num_scenarios, num_algs); % 95% CI half-width (오차 막대 길이)
for k = 1:num_scenarios
    losses = all_losses{k};
    all_std_dev(k, :) = std(losses, 0, 1);
    all_sem(k, :) = all_std_dev(k, :) / sqrt(n);
    all_ci_half_width(k, :) = t_value * all_sem(k, :);
end
scenario_titles = {
    '[시나리오 0] Deterministic',
    '[시나리오 1] 센서 노이즈',
    '[시나리오 2] 모델 불확실성',
    '[시나리오 3] 제어 오차'
};
% -----------------------------------------------------------------
% ----- [MODIFIED] 결과 시각화 (95% 신뢰구간 포함) -----
% -----------------------------------------------------------------
figure('Name', 'IK Algorithm Comparison (95% CI)');
for k = 1:num_scenarios
    subplot(2, 2, k);
    
    % 1. Bar plot (평균값)
    b = bar(all_mean_losses(k, :));
    hold on;
    
    % 2. Add error bars (95% 신뢰구간)
    x_coords = b.XEndPoints; % 막대 중심의 x 좌표 얻기
    y_values = all_mean_losses(k, :);
    errors = all_ci_half_width(k, :); % 95% CI 절반 폭
    
    % 'k.' : 검은색(k) 점(.) 스타일로 오차 막대 상/하단 표시
    errorbar(x_coords, y_values, errors, 'k.', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    hold off;
    
    % 3. Styling
    set(gca, 'XTickLabel', alg_names);
    if k == 1 || k == 3
        ylabel('평균 오차 (Loss)');
    end
    title(scenario_titles{k});
    grid on;
end
sgtitle('IK 방법별 불확실성 시나리오 성능 비교 (95% 신뢰구간)');
%%
% -----------------------------------------------------------------
% ----- [EXISTING] 최종 자세 시각화 (Scenario 0, Trial 1) -----
% -----------------------------------------------------------------
fprintf('\n[시각화] Scenario 0 (Deterministic)의 첫 번째 Trial 결과를 시각화합니다...\n');
figure('Name', 'IK Final Pose Visualization');
% 공통 시각화 설정
view_angles = [145, 25]; % [수평, 수직]
axis_limits = [-1 1 -1 1 -0.2 1.5]; % [xmin xmax ymin ymax zmin zmax]
% Gripper joints는 homeConfiguration 값을 사용
% (stochastic solver들은 7-joint만 반환하므로)
theta_gripper = theta0_full(8:9);
for j = 1:num_algs
    % --- 알고리즘 재실행 (Trial 1 기준) ---
    solver_func = alg_handles{j};
    if j == 1 % Built-in solver (Baseline)
        [theta_sol, ~] = solver_func(x_d_true, theta0);
        theta_7_joints = theta_sol(1:7);
    else % Stochastic solvers
        [theta_7_joints, ~] = solver_func(x_d_true, theta0, max_iter);
    end
    
    % --- 시각화를 위한 전체 configuration (9-joint) 생성 ---
    % (7 arm joints + 2 gripper joints)
    % 'panda'가 'column' DataFormat으로 로드되었으므로 'column' 벡터를 전달
    
    % [FIXED] Removed the transpose (') to create a column vector
    config_full_col = [theta_7_joints; theta_gripper];
    
    % --- Subplot에 그리기 ---
    subplot(2, 2, j);
    
    % [FIXED] Passed the column vector 'config_full_col'
    show(panda, config_full_col, 'PreservePlot', false);
    
    view(view_angles);
    axis(axis_limits);
    title(alg_names{j});
    grid on;
end
sgtitle('Final IK Pose (Scenario 0, Trial 1)');
fprintf('시각화 완료.\n');