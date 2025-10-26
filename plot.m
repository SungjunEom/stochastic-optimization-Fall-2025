%% IK 비교 실험 결과 플로팅 스크립트
%
% 'run_ik_comparison_with_save.m' 스크립트 실행 후 생성된
% 'ik_comparison_results.mat' 파일을 읽어와 그래프를 그립니다.
clc;
clear;
close all;
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
% ----- 결과 시각화 (95% 신뢰구간 포함) -----
% -----------------------------------------------------------------
fprintf('결과 플로팅 중...\n');
figure('Name', 'IK Algorithm Comparison (95% CI)');
num_scenarios = size(all_mean_losses, 1);
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
fprintf('플로팅 완료.\n');
