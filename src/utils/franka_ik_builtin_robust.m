% --- 내장 IK를 여러 번 시도하여 가장 좋은 결과를 선택하는 함수 ---
function [theta_sol, best_loss] = franka_ik_builtin_robust(x_d, theta0, num_attempts, noise_level)
    % Noisy observation 환경에서 built-in solver를 공정하게 비교하기 위한 함수
    %
    % Built-in solver는 내부적으로 고정된 loss function을 사용하므로,
    % 매 iteration마다 noisy observation을 받을 수 없습니다.
    % 대신, 여러 번 시도하여 가장 좋은 결과를 선택합니다.
    %
    % 참고: 이 방법은 built-in solver에게 유리한 조건입니다.
    %       stochastic optimizer들은 매 iteration마다 noisy loss를 평가하지만,
    %       built-in은 여러 번의 깨끗한 시도 중 최선을 선택할 수 있습니다.
    %
    % Inputs:
    %   x_d (6x1):         목표 엔드이펙터 포즈
    %   theta0 (7x1 or 9x1): 초기 관절각
    %   num_attempts (int): 시도 횟수 (default: 5)
    %   noise_level (scalar): 최종 평가 시 노이즈 레벨 (default: 0)
    %
    % Outputs:
    %   theta_sol (9x1): 최선의 관절각 해
    %   best_loss (scalar): 최선의 loss 값
    
    if nargin < 3
        num_attempts = 5;
    end
    if nargin < 4
        noise_level = 0;
    end
    
    panda = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
    ik = inverseKinematics('RigidBodyTree', panda);
    weights = [0.5 0.5 0.5 1 1 1];
    T_goal = trvec2tform(x_d(1:3)') * eul2tform(x_d(4:6)', 'ZYX');
    
    % theta0이 7x1일 경우 9x1로 확장
    if length(theta0) == 7
        theta0 = [theta0; 0; 0];
    end
    
    best_loss = inf;
    theta_sol = theta0;
    
    % 여러 번 시도하여 가장 좋은 결과 선택
    for attempt = 1:num_attempts
        % 각 시도마다 약간 다른 초기값 사용 (exploration 증가)
        if attempt == 1
            theta_init = theta0;
        else
            % 작은 랜덤 perturbation 추가
            theta_init = theta0 + 0.01 * randn(size(theta0));
        end
        
        % Built-in IK 수행
        [theta_candidate, ~] = ik('panda_hand', T_goal, weights, theta_init);
        
        % Noisy observation으로 평가
        loss_candidate = ik_loss_noisy(theta_candidate(1:7), x_d, noise_level);
        
        % 더 좋은 결과면 업데이트
        if loss_candidate < best_loss
            best_loss = loss_candidate;
            theta_sol = theta_candidate;
        end
    end
end

