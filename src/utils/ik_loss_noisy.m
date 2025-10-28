function loss = ik_loss_noisy(theta, x_d, noise_level)
% --- 손실 함수 (Noisy Observation 버전) ---
%
% 이 함수는 noisy observation 환경을 시뮬레이션합니다.
% 매 evaluation마다 센서 노이즈가 추가되어, 동일한 theta에 대해서도
% 매번 다른 loss 값을 반환합니다.
%
% Inputs:
%   theta (7x1):       현재 관절 각도 (radians)
%   x_d (6x1):         목표 엔드이펙터 포즈 [x; y; z; roll; pitch; yaw]
%   noise_level (scalar): 관측 노이즈 표준편차 (default: 0)
%
% Output:
%   loss (scalar): 노이즈가 추가된 손실 값

    % --- 디폴트 noise_level ---
    if nargin < 3
        noise_level = 0;  % 노이즈 없음
    end
    
    % --- 1. Forward Kinematics with Observation Noise ---
    % 센서가 현재 위치를 노이즈와 함께 관측
    x_current_clean = franka_forward_kinematics(theta);
    x_current_observed = x_current_clean + noise_level * randn(6,1);
    
    % --- 2. Pose Error (관측된 위치와 목표 위치 차이) ---
    pose_error = norm(x_current_observed - x_d)^2;
    
    % --- 3. 최종 손실 (정규화 없음) ---
    loss = pose_error;

end

