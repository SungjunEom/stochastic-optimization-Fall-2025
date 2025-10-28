% Gradient-based IK using Numerical Gradient (Finite Difference)
function [theta_opt, history] = gradient_descent_ik(x_d, theta0, max_iter, learning_rate, noise_level)
    % Gradient descent with numerical gradient for inverse kinematics
    %
    % 이 함수는 noisy observation 환경에서 gradient-based optimization의
    % 문제점을 보여주기 위해 구현되었습니다.
    %
    % Inputs:
    %   x_d (6x1):          목표 엔드이펙터 포즈
    %   theta0 (7x1):       초기 관절각
    %   max_iter (int):     최대 반복 횟수
    %   learning_rate (scalar): 학습률 (step size)
    %   noise_level (scalar):  관측 노이즈 레벨 (default: 0)
    %
    % Outputs:
    %   theta_opt (7x1):    최적화된 관절각
    %   history (max_iter x 7): 최적화 과정 기록
    
    if nargin < 5
        noise_level = 0;
    end
    if nargin < 4
        learning_rate = 0.01;  % default learning rate
    end
    
    % 관절 한계
    q_min = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];
    q_max = [ 2.8973;  1.7628;  2.8973; -0.0698;  2.8973;  3.7525;  2.8973];
    
    % 초기화
    theta = clamp_to_limits(theta0, q_min, q_max);
    history = zeros(max_iter, length(theta));
    
    % Finite difference parameter
    epsilon = 1e-6;
    
    % Gradient descent with momentum (optional)
    use_momentum = true;
    momentum = 0.9;
    velocity = zeros(size(theta));
    
    % Learning rate decay
    lr = learning_rate;
    lr_decay = 0.999;  % 천천히 감소
    
    for k = 1:max_iter
        % Numerical gradient 계산 (Finite Difference)
        grad = zeros(size(theta));
        
        % 중심 loss 값 계산 (noisy observation)
        L_center = ik_loss_noisy(theta, x_d, noise_level);
        
        for i = 1:length(theta)
            % Forward difference
            theta_perturb = theta;
            theta_perturb(i) = theta_perturb(i) + epsilon;
            theta_perturb = clamp_to_limits(theta_perturb, q_min, q_max);
            
            L_perturb = ik_loss_noisy(theta_perturb, x_d, noise_level);
            
            % Gradient 계산
            grad(i) = (L_perturb - L_center) / epsilon;
        end
        
        % Gradient descent update
        if use_momentum
            % Momentum-based update (noisy gradient에서 더 안정적)
            velocity = momentum * velocity - lr * grad;
            theta_new = theta + velocity;
        else
            % Vanilla gradient descent
            theta_new = theta - lr * grad;
        end
        
        % 관절 한계 적용
        theta = clamp_to_limits(theta_new, q_min, q_max);
        
        % Learning rate decay
        lr = lr * lr_decay;
        
        % 기록
        history(k, :) = theta;
        
        % Early stopping (optional)
        if L_center < 1e-6
            history(k+1:end, :) = repmat(theta', max_iter - k, 1);
            break;
        end
    end
    
    theta_opt = theta;
end

