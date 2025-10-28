% 프랑카 판다 역기구학을 위한 확률적 최적화 (Simple Random Search)
function [theta_opt, history] = simple_random_search(x_d, theta0, max_iter, rho)
    % x_d: 목표 엔드이펙터 위치 (6x1 벡터: 위치 + 자세)
    % theta0: 초기 관절각 추정치 (7x1)
    % max_iter: 최대 반복 횟수

    % --- 로봇 관절 한계 (알고리즘의 'Θ') ---
    q_min = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];
    q_max = [ 2.8973;  1.7628;  2.8973; -0.0698;  2.8973;  3.7525;  2.8973];
    q_range = q_max - q_min; % 탐색 범위를 미리 계산

    % --- 초기화 ---
    % 현재 최고해 (θ̂k)
    theta_hat = clamp_to_limits(theta0, q_min, q_max); 
    history = zeros(max_iter, length(theta_hat));
    
    % 현재 최고 손실값 (L(θ̂k))
    L_k = ik_loss(theta_hat, x_d);
    
    for k = 1:max_iter
        % (Step 1) 도메인 'Θ' 전체에서 새로운 랜덤 후보 생성
        % (q_min, q_max) 범위 내에서 균일 분포로 샘플링
        theta_new = q_min + q_range .* rand(size(theta_hat));
        
        % (Step 2) 새 후보 평가
        L_new = ik_loss(theta_new, x_d);
        
        % (Step 3) 개선 여부 확인
        if L_new < L_k
            theta_hat = theta_new;   % θ̂k+1 ← θnew
            L_k = L_new;             % 최고 손실값 업데이트
        end
        % else: theta_hat은 변경되지 않음 (θ̂k+1 ← θ̂k)
        
        history(k, :) = theta_hat;
    end
    
    theta_opt = theta_hat;
end