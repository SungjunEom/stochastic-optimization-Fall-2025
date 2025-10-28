% Adam Optimizer for IK (Gradient-based with Adaptive Learning Rate)
function [theta_opt, history] = adam_ik(x_d, theta0, max_iter, learning_rate, noise_level)
    % Adam optimizer (Adaptive Moment Estimation) for inverse kinematics
    %
    % Adam은 gradient의 1차, 2차 모멘트를 사용하여 adaptive learning rate를 제공합니다.
    % Noisy gradient 환경에서 vanilla gradient descent보다 robust합니다.
    %
    % Inputs:
    %   x_d (6x1):          목표 엔드이펙터 포즈
    %   theta0 (7x1):       초기 관절각
    %   max_iter (int):     최대 반복 횟수
    %   learning_rate (scalar): 초기 학습률 (default: 0.01)
    %   noise_level (scalar):  관측 노이즈 레벨 (default: 0)
    %
    % Outputs:
    %   theta_opt (7x1):    최적화된 관절각
    %   history (max_iter x 7): 최적화 과정 기록
    
    if nargin < 5
        noise_level = 0;
    end
    if nargin < 4
        learning_rate = 0.01;
    end
    
    % 관절 한계
    q_min = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];
    q_max = [ 2.8973;  1.7628;  2.8973; -0.0698;  2.8973;  3.7525;  2.8973];
    
    % 초기화
    theta = clamp_to_limits(theta0, q_min, q_max);
    history = zeros(max_iter, length(theta));
    
    % Adam 파라미터
    beta1 = 0.9;    % 1차 모멘트 decay rate
    beta2 = 0.999;  % 2차 모멘트 decay rate
    eps_adam = 1e-8;  % numerical stability
    
    m = zeros(size(theta));  % 1st moment (mean of gradients)
    v = zeros(size(theta));  % 2nd moment (variance of gradients)
    
    % Finite difference parameter
    epsilon = 1e-6;
    
    for k = 1:max_iter
        % Numerical gradient 계산
        grad = zeros(size(theta));
        L_center = ik_loss_noisy(theta, x_d, noise_level);
        
        for i = 1:length(theta)
            theta_perturb = theta;
            theta_perturb(i) = theta_perturb(i) + epsilon;
            theta_perturb = clamp_to_limits(theta_perturb, q_min, q_max);
            
            L_perturb = ik_loss_noisy(theta_perturb, x_d, noise_level);
            grad(i) = (L_perturb - L_center) / epsilon;
        end
        
        % Adam update
        m = beta1 * m + (1 - beta1) * grad;
        v = beta2 * v + (1 - beta2) * (grad .^ 2);
        
        % Bias correction
        m_hat = m / (1 - beta1^k);
        v_hat = v / (1 - beta2^k);
        
        % Parameter update
        theta_new = theta - learning_rate * m_hat ./ (sqrt(v_hat) + eps_adam);
        
        % 관절 한계 적용
        theta = clamp_to_limits(theta_new, q_min, q_max);
        
        % 기록
        history(k, :) = theta;
        
        % Early stopping
        if L_center < 1e-6
            history(k+1:end, :) = repmat(theta', max_iter - k, 1);
            break;
        end
    end
    
    theta_opt = theta;
end

