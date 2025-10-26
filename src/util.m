% 1. 'frankaEmikaPanda' 로봇 모델을 불러옵니다.
panda = loadrobot("frankaEmikaPanda");

% 2. 관절 각도 제한 속성을 가져옵니다.
%    이 속성은 N x 2 행렬이며, 각 행은 [최소값, 최대값] 입니다.
jointLimits = panda.JointPositionLimits;

% 3. 관절 이름을 확인하기 위해 로봇의 기본 설정을 가져옵니다.
%    JointPositionLimits의 행 순서는 homeConfiguration의 관절 순서와 일치합니다.
config = panda.homeConfiguration;

disp("Franka Emika Panda 관절 각도 제한 (단위: 라디안):");
disp("--------------------------------------------------");

% 4. 각 관절의 이름과 제한 값을 함께 표시합니다.
for i = 1:numel(config)
    jointName = config(i).JointName;
    minLimit = jointLimits(i, 1);
    maxLimit = jointLimits(i, 2);
    
    % %dof(자유도)가 0인 joint(예: 'panda_hand')는 제외합니다.
    if ~isempty(minLimit) 
        fprintf('%-15s: [최소: %+.4f, 최대: %+.4f]\n', jointName, minLimit, maxLimit);
    end
end