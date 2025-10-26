function loss = ik_loss(theta, x_d)
% --- 손실 함수: (목표 포즈 오차) + (관절 정규화 항) ---
%
% 이 함수는 엔드이펙터 포즈(위치+방향) 오차와
% 관절 각도의 정규화(regularization) 항을 더하여 최종 손실을 계산합니다.
%
% Inputs:
%   theta (7x1): 현재 관절 각도 (radians)
%   x_d (6x1):   목표 엔드이펙터 포즈 [x; y; z; roll; pitch; yaw]
%
% Output:
%   loss (scalar): 최종 손실 값

    % --- 1. 포즈(Pose) 오차 계산 ---
    % franka_forward_kinematics가 6x1 (위치+방향) 벡터를 반환한다고 가정합니다.
    x_current = franka_forward_kinematics(theta);
    
    % x_d와 x_current는 모두 6x1 벡터여야 합니다.
    % (e.g., [pos_x, pos_y, pos_z, orient_r, orient_p, orient_y])
    pose_error = norm(x_current - x_d)^2;

    % --- 2. 관절 정규화(Regularization) 오차 계산 ---
    
    % 'frankaEmikaPanda'의 'homeConfiguration' (상위 7개 관절)
    % 이 값은 메인 스크립트에서 정의된 theta0 (homeConfiguration)과 동일합니다.
    theta_home = [
         0.0;
        -0.7854;  % (=-pi/4)
         0.0;
        -2.3562;  % (=-3*pi/4)
         0.0;
         1.5708;  % (=pi/2)
         0.7854   % (=pi/4)
    ];
    
    % 정규화 가중치 (Tuning 필요)
    % - 이 값이 크면: 로봇이 'theta_home' 자세를 유지하려 하고 목표 도달을 포기할 수 있습니다.
    % - 이 값이 작으면: 정규화 효과가 미미합니다.
    % 1e-2, 1e-3, 1e-4 등 여러 값으로 테스트해보세요.
    w = 1e-3; 

    % 관절 공간에서의 오차 (기준 자세와의 거리)
    joint_regularization_error = norm(theta - theta_home)^2;

    % --- 3. 최종 손실 ---
    loss = pose_error + w * joint_regularization_error;

end
