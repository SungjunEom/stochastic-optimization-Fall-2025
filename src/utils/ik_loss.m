
% --- 손실 함수: 목표 엔드이펙터 위치와의 차이 제곱합 ---
function loss = ik_loss(theta, x_d)
    x = franka_forward_kinematics(theta);  % FK로 엔드이펙터 위치 계산 (사용자 정의 필요)
    loss = norm(x - x_d)^2;
end
