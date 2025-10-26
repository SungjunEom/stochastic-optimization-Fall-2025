% 프랑카 판다 역기구학을 위한 확률적 최적화 (Localized Random Search)
function [theta_opt, history] = localized_random_search(x_d, theta0, max_iter)
    % x_d: 목표 엔드이펙터 위치 (6x1 벡터: 위치 + 자세)
    % theta0: 초기 관절각 추정치 (7x1)
    % max_iter: 최대 반복 횟수

    % --- 로봇 관절 한계 (알고리즘의 'Θ') ---
    q_min = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];
    q_max = [ 2.8973;  1.7628;  2.8973; -0.0698;  2.8973;  3.7525;  2.8973];

    % --- 하이퍼파라미터 ---
    rho = 0.1;       % 랜덤 탐색 범위 (d_k 생성용)
    
    % --- 초기화 ---
    % (Step 4의 θ̂k)
    % 시작점이 유효한지 확인
    theta_hat = clamp_to_limits(theta0, q_min, q_max); 
    history = zeros(max_iter, length(theta_hat));
    
    % (Step 4의 L(θ̂k))
    % 현재(최고) 손실값을 캐싱하여 불필요한 계산 방지
    L_k = ik_loss(theta_hat, x_d);
    
    for k = 1:max_iter
        % (Step 3) Generate a zero-mean r.v. dk
        d = rho * randn(size(theta_hat));
        
        % (Step 5) 첫 번째 후보 생성 (θ̂k + dk) 및 제약 조건 처리
        theta_try_1 = theta_hat + d;
        theta_new_1 = clamp_to_limits(theta_try_1, q_min, q_max);
        
        % (Step 6) 첫 번째 후보 평가
        L_new_1 = ik_loss(theta_new_1, x_d);
        if L_new_1 < L_k
            theta_hat = theta_new_1;   % θ̂k+1 ← θnew
            L_k = L_new_1;             % 다음 비교를 위해 현재 손실 업데이트
        else
            % (Step 7) 두 번째 후보 생성 (θ̂k - dk) 및 제약 조건 처리
            theta_try_2 = theta_hat - d;
            theta_new_2 = clamp_to_limits(theta_try_2, q_min, q_max);
            
            % (Step 8) 두 번째 후보 평가
            L_new_2 = ik_loss(theta_new_2, x_d);
            if L_new_2 < L_k
                theta_hat = theta_new_2;   % θ̂k+1 ← θnew
                L_k = L_new_2;             % 다음 비교를 위해 현재 손실 업데이트
            end
            % (Step 9) 둘 다 실패하면 아무것도 하지 않음 (θ̂k+1 ← θ̂k)
        end
        
        history(k, :) = theta_hat;
    end
    
    theta_opt = theta_hat;
end