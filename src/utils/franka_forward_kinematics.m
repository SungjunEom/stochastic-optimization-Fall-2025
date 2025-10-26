function x = franka_forward_kinematics(theta)
    % 입력: theta - 7x1 관절각 (rad)
    % 출력: x - 6x1 벡터 (위치 3 + 오일러각 3)

    % 로봇 모델 불러오기
    persistent panda
    if isempty(panda)
        panda = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');
    end

    % theta가 7x1이면 9x1로 확장 (gripper 조인트는 0으로 설정)
    if length(theta) == 7
        theta = [theta; 0; 0];
    end

    % FK 계산
    T = getTransform(panda, theta, 'panda_hand');
    pos = tform2trvec(T)';
    R = tform2rotm(T);
    eul = rotm2eul(R, 'ZYX')';

    x = [pos; eul];
end
