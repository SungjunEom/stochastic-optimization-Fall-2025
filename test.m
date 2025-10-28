%% Franka Emika Panda IK 비교 실험 통합 스크립트 (데이터 저장용)
%
% 이 스크립트는 4개의 알고리즘을 4개의 시나리오에 대해 비교하고,
% 플로팅을 위해 결과를 'ik_comparison_results.mat' 파일로 저장합니다.
%
% [알고리즘]
% 1. Gradient Descent (GD) - Numerical gradient with momentum
% 2. Adam Optimizer (Adam) - Adaptive learning rate
% 3. Simple Random Search (SRS)
% 4. Localized Random Search (LRS)
% 5. Enhanced Localized Random Search (ELRS)
%
% [시나리오]
% 0. Deterministic (노이즈 없음)
% 1. Noisy Observation (매 iteration마다 센서 노이즈)
% 2. FK 모델 불확실성 (최적화 중 FK에 오차)
% 3. 조인트 제어 오차 (최적화 후 제어 오차)
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
max_iter = 1000; % 최적화 반복 횟수
rho = 0.005; % for stochastic algorithms
learning_rate = 0.01; % for gradient-based algorithms
sigma = 0.005; % observation noise level (5mm position, ~0.3 deg orientation)
% --- 알고리즘 정의 ---
% 함수 핸들을 셀 배열로 정의
alg_handles = {
    @gradient_descent_ik, ...
    @adam_ik, ...
    @simple_random_search, ...
    @localized_random_search, ...
    @enhanced_localized_random_search
};
% 그래프 및 출력용 이름
alg_names = {'GD', 'Adam', 'SRS', 'LRS', 'ELRS'};
num_algs = length(alg_names);
num_scenarios = 4; % 시나리오 개수
fprintf('프랑카 판다 로봇 IK 비교 실험을 시작합니다.\n');
fprintf('알고리즘: %s\n', strjoin(alg_names, ', '));
fprintf('============================================\n');
%% 시나리오 0: Deterministic (노이즈 없음)
fprintf('\n[시나리오 0: Deterministic] 실험 중...\n');
losses_s0 = zeros(num_trials, num_algs);
for i = 1:num_trials
    for j = 1:num_algs
        solver_func = alg_handles{j};
        
        if j <= 2 % Gradient-based solvers (GD, Adam)
            % noise_level = 0 (노이즈 없음)
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter, learning_rate, 0);
        else % Stochastic solvers (SRS, LRS, ELRS)
            % noise_level = 0 (노이즈 없음)
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter, rho, 0);
        end
        
        % 실제 목표 위치(x_d_true)와의 오차 계산 (deterministic)
        losses_s0(i, j) = ik_loss(theta_j, x_d_true);
    end
end
mean_losses_s0 = mean(losses_s0, 1);
for j = 1:num_algs
    fprintf('▶ %-10s mean loss: %.6f\n', [alg_names{j} ':'], mean_losses_s0(j));
end
%% 시나리오 1: Noisy Observation (센서 노이즈)
% 의미: "매 iteration마다 센서가 현재 위치를 노이즈와 함께 관측"
% 진짜 noisy observation! Gradient-based는 noisy gradient로 고생할 것
fprintf('\n[시나리오 1: Noisy Observation] 실험 중...\n');
losses_s1 = zeros(num_trials, num_algs);
for i = 1:num_trials
    for j = 1:num_algs
        solver_func = alg_handles{j};
        
        if j <= 2 % Gradient-based solvers
            % 매 iteration마다 noisy gradient 계산
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter, learning_rate, sigma);
        else % Stochastic solvers
            % 매 iteration마다 noisy observation 사용
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter, rho, sigma);
        end
        
        % 최종 평가: 실제 목표 위치와의 오차 (deterministic)
        losses_s1(i, j) = ik_loss(theta_j, x_d_true);
    end
end
mean_losses_s1 = mean(losses_s1, 1);
for j = 1:num_algs
    fprintf('▶ %-10s mean loss: %.6f\n', [alg_names{j} ':'], mean_losses_s1(j));
end
%% 시나리오 2: FK 모델 불확실성
% 의미: "최적화 과정에서 FK 모델에 오차가 있음"
% 시나리오 1과 동일하게 noisy observation을 사용 (동일한 효과)
fprintf('\n[시나리오 2: FK 모델 불확실성] 실험 중...\n');
losses_s2 = zeros(num_trials, num_algs);
for i = 1:num_trials
    for j = 1:num_algs
        solver_func = alg_handles{j};
        
        if j <= 2 % Gradient-based solvers
            % FK 모델 오차로 인한 noisy gradient
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter, learning_rate, sigma);
        else % Stochastic solvers
            % FK 모델 오차로 인한 noisy observation
            [theta_j, ~] = solver_func(x_d_true, theta0, max_iter, rho, sigma);
        end
        
        % 최종 평가: 실제 목표 위치와의 오차 (deterministic)
        losses_s2(i, j) = ik_loss(theta_j, x_d_true);
    end
end
mean_losses_s2 = mean(losses_s2, 1);
for j = 1:num_algs
    fprintf('▶ %-10s mean loss: %.6f\n', [alg_names{j} ':'], mean_losses_s2(j));
end
%% 시나리오 3: 조인트 제어 오차
% 의미: "계산된 관절각 θ를 로봇이 정확히 따라가지 못함"
% 주의: 이 노이즈는 최적화 후에만 적용됨 (최적화 중엔 의미 없음)
fprintf('\n[시나리오 3: 조인트 제어 오차] 실험 중...\n');
losses_s3 = zeros(num_trials, num_algs);
for i = 1:num_trials
    
    theta_solutions = cell(num_algs, 1);
    
    % 1. 모든 솔버로 θ 계산 (깨끗한 환경에서 최적화)
    for j = 1:num_algs
        solver_func = alg_handles{j};
        if j <= 2 % Gradient-based solvers
            % noise_level = 0 (제어 노이즈는 나중에 추가됨)
            [theta_solutions{j}, ~] = solver_func(x_d_true, theta0, max_iter, learning_rate, 0);
        else % Stochastic solvers
            % noise_level = 0 (제어 노이즈는 나중에 추가됨)
            [theta_solutions{j}, ~] = solver_func(x_d_true, theta0, max_iter, rho, 0);
        end
    end
    
    % 2. 동일한 제어 노이즈를 적용하여 실제 위치 계산 및 오차 평가
    control_noise = deg2rad(1.0) * randn(7,1); % 1도 정도의 제어 오차
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
    '[시나리오 1] Noisy Observation',
    '[시나리오 2] FK 모델 불확실성',
    '[시나리오 3] 조인트 제어 오차'
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
