%% Franka Emika Panda IK 비교 실험 통합 스크립트 (데이터 저장용)
%
% 이 스크립트는 4개의 알고리즘을 4개의 시나리오에 대해 비교하고,
% 플로팅을 위해 결과를 'ik_comparison_results.mat' 파일로 저장합니다.
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
%
% [수정]
% - 시각화/플로팅 코드를 제거했습니다.
% - 스크립트 마지막에 통계 결과를 .mat 파일로 저장하는 기능을 추가했습니다.
%
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
rho = 0.05; % for algorithms
sigma = 0.005; % for noise
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
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter, rho);
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
    obs_noise = sigma * randn(6,1);
    x_d_noisy = x_d_true + obs_noise;
    for j = 1:num_algs
        solver_func = alg_handles{j};
        
        if j == 1 % Built-in solver
            [theta_sol, ~] = solver_func(x_d_noisy, theta0);
            theta_j = theta_sol(1:7);
        else % Stochastic solvers
            [theta_j, ~] = solver_func(x_d_noisy, theta0, max_iter, rho);
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
            [theta_solutions{j}, ~] = solver_func(x_d_true, theta0, max_iter, rho);
        end
    end
    % 2. 동일한 FK 노이즈를 적용하여 실제 위치 계산 및 오차 평가
    fk_noise = sigma * randn(6,1);
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
            [theta_solutions{j}, ~] = solver_func(x_d_true, theta0, max_iter, rho);
        end
    end
    % 2. 동일한 제어 노이즈를 적용하여 실제 위치 계산 및 오차 평가
    control_noise = deg2rad(1.0) * randn(7,1); % original
    % control_noise = deg2rad(sigma) * randn(7,1);
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
% 시나리오 제목 (플로팅 스크립트에서 사용)
scenario_titles = {
    '[시나리오 0] Deterministic',
    '[시나리오 1] 센서 노이즈',
    '[시나리오 2] 모델 불확실성',
    '[시나리오 3] 제어 오차'
};
fprintf('통계 계산 완료.\n');
% -----------------------------------------------------------------
% ----- 결과 데이터 파일로 저장 -----
% -----------------------------------------------------------------
results_filename = 'ik_comparison_results.mat';
try
    save(results_filename, ...
        'all_losses', ...
        'all_mean_losses', ...
        'all_ci_half_width', ...
        'alg_names', ...
        'scenario_titles', ...
        'num_trials');
    
    fprintf('\n[데이터 저장] 성공!\n');
    fprintf('결과가 %s 파일에 저장되었습니다.\n', results_filename);
    
catch e
    fprintf('\n[데이터 저장] 실패!\n');
    fprintf('오류 메시지: %s\n', e.message);
end
fprintf('============================================\n');
fprintf('시뮬레이션 완료.\n');
