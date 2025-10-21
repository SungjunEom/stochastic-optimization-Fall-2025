% --- 내장 IK를 이용한 비교 함수 ---
function [theta_sol, sol_info] = franka_ik_builtin(x_d, theta0)
    panda = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
    ik = inverseKinematics('RigidBodyTree', panda);

    weights = [0.5 0.5 0.5 1 1 1]; % 위치와 자세에 대한 중요도 조절
    T_goal = trvec2tform(x_d(1:3)') * eul2tform(x_d(4:6)', 'ZYX');

    % theta0이 7x1일 경우 9x1로 확장
    if length(theta0) == 7
        theta0 = [theta0; 0; 0];  % 그리퍼 조인트 0으로 설정
    end

    [theta_sol, sol_info] = ik('panda_hand', T_goal, weights, theta0);
end
