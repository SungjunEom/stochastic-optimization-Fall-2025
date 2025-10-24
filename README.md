# 프랑카 판다 로봇의 확률적 역기구학 비교 실험

이 프로젝트는 7자유도 로봇팔인 **Franka Emika Panda**의 **역기구학(Inverse Kinematics)** 문제를 **확률적 최적화 기법**으로 풀고, MATLAB 기본 제공 역기구학 해석기와 비교하여 실제 환경에서 어떤 방식이 더 강건한지(robust) 평가하는 실험입니다.

---

## 🤖 역기구학이란?

로봇팔의 목표 위치가 주어졌을 때, 그 위치에 도달하기 위한 **관절 각도 θ**들을 계산하는 문제입니다.

예)  
> "로봇 손 끝이 어떤 좌표에 도달해야 할 때, 팔 관절들을 어떻게 움직이면 될까?"

---

## 🔍 실험 목적

실제 로봇 시스템은 아래와 같은 **불확실성**을 포함합니다:

- **센서 노이즈**: 실제 위치를 잘못 측정할 수 있음 (예: 카메라 오차)
- **모델 오차**: 로봇 자체 모델이 완벽하지 않음 (캘리브레이션 오류)
- **제어 오차**: 계산된 각도를 정확히 구현하지 못함 (모터 제어 오차)

이러한 상황에서 전통적인 IK 방법보다 **확률적 방법이 더 효과적인지**를 실험합니다.

---

## 🗂️ 파일 구성

| 파일명                        | 설명 |
|------------------------------|------|
| `franka_ik_stochastic.m`     | 확률적 IK 최적화 함수 (Localized Random Search 기반) | (추가 구현 필요 부분)
| `franka_ik_builtin.m`        | MATLAB 내장 역기구학 함수 래퍼 |
| `franka_forward_kinematics.m`| 주어진 관절 각도로 위치 계산 (순기구학) |
| `test.m`                     | 기본 비교 실험 실행 스크립트 |
| `scenario_2_and_3.m`         | 모델 오차, 제어기 오차 시나리오 평가 코드 |
| `scenario_all_combined.m`    | 모든 불확실성(1+2+3)을 동시에 적용한 통합 실험 |

---

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

