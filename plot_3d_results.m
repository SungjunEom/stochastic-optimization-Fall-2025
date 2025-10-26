%% Franka Emika 로봇 자세 시각화 스크립트 (시나리오 선택 가능)
%
% 'run_ik_comparison_with_save.m' 실행과 별개로,
% 선택한 시나리오의 첫 번째 Trial 결과를
% 재계산하여 시각화합니다.
%
% 이 스크립트를 실행하기 전에 'src' 폴더가 경로에 추가되어 있어야 합니다.
clc;
clear;
close all;
% -----------------------------------------------------------------
% ----- [NEW] 시각화할 시나리오 선택 -----
% -----------------------------------------------------------------
% 0: Deterministic
% 1: 센서 노이즈 (Sensor Noise)
% 2: 모델 불확실성 (Model Uncertainty)
% 3: 제어 오차 (Control Noise)
scenario_to_visualize = 0;
% -----------------------------------------------------------------
% --- Add 'src' and all its subfolders to the path ---
% 이 스크립트가 메인 스크립트와 동일한 폴더에 있다고 가정합니다.
scriptDir = fileparts(mfilename('fullpath'));
srcDir = fullfile(scriptDir, 'src');
if ~exist(srcDir, 'dir')
    warning('"%s" 폴더를 찾을 수 없습니다. IK 함수를 불러오지 못할 수 있습니다.', srcDir);
else
    addpath(genpath(srcDir));
    fprintf('src 폴더를 경로에 추가했습니다.\n');
end
%% 공통 파라미터 설정 (시각화에 필요한 부분만)
x_d_true = [0.4; 0.2; 0.3; 0; 0; 0]; % 실제 목표 위치 (Ground Truth)
panda = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
theta0_full = homeConfiguration(panda);
theta0 = theta0_full(1:7); % 7개 관절만 사용
max_iter = 1000; % Stochastic IK 반복 횟수 (원본과 동일하게)
% --- 알고리즘 정의 ---
alg_handles = {
    @franka_ik_builtin, ...
    @simple_random_search, ...
    @localized_random_search, ...
    @enhanced_localized_random_search
};
alg_names = {'Built-in', 'SRS', 'LRS', 'ELRS'};
num_algs = length(alg_names);
% -----------------------------------------------------------------
% ----- [NEW] 시나리오별 파라미터 설정 -----
% -----------------------------------------------------------------
scenario_title_str = '';
target_pose_for_solver = x_d_true;
control_noise_for_viz = zeros(7,1); % 기본값: 제어 노이즈 없음
switch scenario_to_visualize
    case 0
        scenario_title_str = 'Scenario 0: Deterministic';
        % target_pose_for_solver는 이미 x_d_true
        % control_noise_for_viz는 이미 0
        
    case 1
        scenario_title_str = 'Scenario 1: Sensor Noise';
        % 모든 알고리즘이 동일한 노이즈가 적용된 목표를 받도록
        % 루프 밖에서 한 번만 노이즈 생성
        obs_noise = 0.005 * randn(6,1);
        target_pose_for_solver = x_d_true + obs_noise;
        
    case 2
        scenario_title_str = 'Scenario 2: Model Uncertainty (Calculated Pose)';
        % 솔버는 x_d_true를 목표로 계산합니다.
        % (평가 시에만 FK 노이즈가 추가되므로, 계산된 자세는 S0와 동일)
        target_pose_for_solver = x_d_true;
        
    case 3
        scenario_title_str = 'Scenario 3: Control Noise (Actual Pose)';
        % 솔버는 x_d_true를 목표로 계산합니다.
        target_pose_for_solver = x_d_true;
        % 하지만 시각화는 제어 노이즈가 추가된 *실제* 자세를 보여줍니다.
        % 모든 알고리즘에 동일한 노이즈 적용
        control_noise_for_viz = deg2rad(1.0) * randn(7,1);
        
    otherwise
        error('잘못된 scenario_to_visualize 값입니다. 0, 1, 2, 3 중 하나여야 합니다.');
end
% -----------------------------------------------------------------
% ----- [MODIFIED] 최종 자세 시각화 -----
% -----------------------------------------------------------------
fprintf('\n[시각화] %s의 첫 번째 Trial 결과를 시각화합니다...\n', scenario_title_str);
figure('Name', ['IK Final Pose Visualization - ' scenario_title_str]);
% 공통 시각화 설정
view_angles = [145, 25]; % [수평, 수직]
axis_limits = [-1 1 -1 1 -0.2 1.5]; % [xmin xmax ymin ymax zmin zmax]
% Gripper joints는 homeConfiguration 값을 사용
theta_gripper = theta0_full(8:9);
for j = 1:num_algs
    % --- 알고리즘 재실행 (Trial 1, 선택된 시나리오 기준) ---
    solver_func = alg_handles{j};
    
    fprintf('  %s 알고리즘 계산 중...\n', alg_names{j});
    
    if j == 1 % Built-in solver (Baseline)
        [theta_sol, ~] = solver_func(target_pose_for_solver, theta0);
        theta_7_joints_intended = theta_sol(1:7);
    else % Stochastic solvers
        [theta_7_joints_intended, ~] = solver_func(target_pose_for_solver, theta0, max_iter);
    end
    
    % --- [NEW] Scenario 3의 경우 제어 노이즈 적용 ---
    theta_7_joints_actual = theta_7_joints_intended + control_noise_for_viz;
    
    % --- 시각화를 위한 전체 configuration (9-joint) 생성 ---
    % [MODIFIED] control_noise가 적용된 실제 자세(theta_7_joints_actual) 사용
    config_full_col = [theta_7_joints_actual; theta_gripper];
    
    % --- Subplot에 그리기 ---
    subplot(2, 2, j);
    show(panda, config_full_col, 'PreservePlot', false);
    
    view(view_angles);
    axis(axis_limits);
    title(alg_names{j});
    grid on;
end
sgtitle(sprintf('Final IK Pose (%s, Trial 1)', scenario_title_str));
fprintf('시각화 완료.\n');

