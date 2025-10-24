# 프랑카 판다 로봇의 확률적 역기구학 비교 실험

이 프로젝트는 7자유도 로봇팔인 **Franka Emika Panda**의 **역기구학(Inverse Kinematics)** 문제를 **확률적 최적화 기법**으로 풀고, MATLAB 기본 제공 역기구학 해석기와 비교하여 실제 환경에서 어떤 방식이 더 강건한지(robust) 평가하는 실험입니다.

## ⚙️ 필요 환경

- MATLAB R2021a 이상
- [Robotics Toolbox](https://www.mathworks.com/help/robotics/) 설치
- `loadrobot("frankaEmikaPanda")` 로봇 모델 사용 가능해야 함

---


## ▶️ 실행 방법

```matlab
test                % 각 상황별 비교(센서 노이즈, 모델 오차, 제어 오차) 및 전체 통합 비교
```

---
## Scenario 1: Deterministic IK

### Methods

- Conventional IK (baseline)
- Simple Random Search
- Localized Random Search
- Enhanced Localized Random Search

## Scenario 2: IK with Noisy Measurements

### Methods

- Conventional IK (baseline)
- Simple Random Search
- Localized Random Search
- Enhanced Localized Random Search

