%% IK 비교 실험 결과 플로팅 스크립트 (선택적 플로팅)
%
% 'run_ik_comparison_with_save.m' 스크립트 실행 후 생성된
% 'ik_comparison_results.mat' 파일을 읽어와 그래프를 그립니다.
%
% [수정]
% - 'algs_to_plot' 변수를 추가하여 원하는 알고리즘만 선택적으로
%   플로팅할 수 있도록 수정했습니다.
clc;
clear;
close all;
% -----------------------------------------------------------------
% ----- [NEW] 플로팅할 알고리즘 선택 -----
% -----------------------------------------------------------------
% 아래 리스트에서 플로팅할 알고리즘 이름만 남기세요.
% algs_to_plot = {
%     'Built-in', ...
%     'SRS', ...
%     'LRS', ...
%     'ELRS'
% };
% 예시: 'Built-in'과 'ELRS'만 비교하고 싶다면:
algs_to_plot = {'Built-in', 'LRS', 'ELRS'};
% -----------------------------------------------------------------
% --- 결과 파일 로드 ---
results_filename = 'ik_comparison_results.mat';
if ~exist(results_filename, 'file')
    error('결과 파일 "%s"을(를) 찾을 수 없습니다.\n먼저 "run_ik_comparison_with_save.m" 스크립트를 실행하세요.', results_filename);
end
fprintf('"%s" 파일에서 데이터를 로드 중...\n', results_filename);
load(results_filename);
% --- 필요한 변수가 로드되었는지 확인 ---
required_vars = {'all_mean_losses', 'all_ci_half_width', 'alg_names', 'scenario_titles'};
for i = 1:length(required_vars)
    if ~exist(required_vars{i}, 'var')
        error('로드된 파일에 "%s" 변수가 없습니다. 데이터 파일이 손상되었거나 잘못되었습니다.', required_vars{i});
    end
end
fprintf('데이터 로드 완료.\n');
% -----------------------------------------------------------------
% ----- [NEW] 선택된 알고리즘 인덱스 찾기 -----
% -----------------------------------------------------------------
[~, plot_indices] = ismember(algs_to_plot, alg_names);
% 0인 인덱스 (일치하는 이름을 못찾은 경우) 제거
plot_indices = plot_indices(plot_indices > 0);
% 정렬 (algs_to_plot 순서가 아닌 alg_names의 원래 순서대로)
plot_indices = sort(plot_indices);
if isempty(plot_indices)
    error('선택된 알고리즘(%s) 중 유효한 이름이 없습니다. ''alg_names''를 확인하세요.', strjoin(algs_to_plot, ', '));
end
% --- 플로팅할 데이터 필터링 ---
mean_losses_filtered = all_mean_losses(:, plot_indices);
ci_half_width_filtered = all_ci_half_width(:, plot_indices);
alg_names_filtered = alg_names(plot_indices);
fprintf('선택된 알고리즘으로 플로팅합니다: %s\n', strjoin(alg_names_filtered, ', '));
% -----------------------------------------------------------------
% ----- [MODIFIED] 결과 시각화 (95% 신뢰구간 포함) -----
% -----------------------------------------------------------------
fprintf('결과 플로팅 중...\n');
figure('Name', 'IK Algorithm Comparison (95% CI)');
num_scenarios = size(all_mean_losses, 1); % 시나리오 개수는 전체 데이터 기준
for k = 1:num_scenarios
    subplot(2, 2, k);
    
    % 1. Bar plot (필터링된 평균값)
    b = bar(mean_losses_filtered(k, :));
    hold on;
    
    % 2. Add error bars (필터링된 95% 신뢰구간)
    x_coords = b.XEndPoints; % 막대 중심의 x 좌표 얻기
    y_values = mean_losses_filtered(k, :);
    errors = ci_half_width_filtered(k, :); % 95% CI 절반 폭
    
    % 'k.' : 검은색(k) 점(.) 스타일로 오차 막대 상/하단 표시
    errorbar(x_coords, y_values, errors, 'k.', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    hold off;
    
    % 3. Styling (필터링된 이름 사용)
    set(gca, 'XTickLabel', alg_names_filtered);
    if k == 1 || k == 3
        ylabel('평균 오차 (Loss)');
    end
    title(scenario_titles{k});
    grid on;
end
sgtitle('IK 방법별 불확실성 시나리오 성능 비교 (95% 신뢰구간)');
fprintf('플로팅 완료.\n');

