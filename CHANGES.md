# 🔧 Noisy Observation 구현 변경사항

## 문제점
기존 코드는 **noisy target**을 테스트하고 있었지, **noisy observation**이 아니었습니다.
- 노이즈가 단 한 번만 추가되어 모든 알고리즘이 동일한 shifted target을 받음
- Loss function이 deterministic해서 확률적 최적화의 의미가 없음
- Built-in solver와 차이가 없는 것이 당연한 상황

## 해결책

### 1. 새로운 Noisy Loss Function 추가
**파일**: `src/utils/ik_loss_noisy.m`
- 매 evaluation마다 센서 노이즈 추가
- 동일한 θ에 대해서도 매번 다른 loss 반환
- 진짜 noisy observation 환경 시뮬레이션

### 2. 확률적 Optimizer 수정
**수정된 파일**:
- `src/optimizers/simple_random_search.m`
- `src/optimizers/localized_random_search.m`
- `src/optimizers/enhanced_localized_random_search.m`

**변경사항**:
- `noise_level` 파라미터 추가 (5번째 인자)
- `ik_loss()` → `ik_loss_noisy()` 사용
- 매 iteration마다 noisy observation 받음

### 3. Built-in Solver Wrapper 추가
**파일**: `src/utils/franka_ik_builtin_robust.m`
- Built-in은 custom loss function을 사용할 수 없음
- 여러 번 시도하여 가장 좋은 결과 선택
- 주의: 이 방법은 built-in에게 유리한 조건

### 4. Test Script 수정
**파일**: `test.m`

**시나리오 1 변경**:
```matlab
% 이전: Noisy target (단 한 번의 노이즈 추가)
x_d_noisy = x_d_true + sigma * randn(6,1);
[theta_j, ~] = solver_func(x_d_noisy, theta0, max_iter, rho);

% 현재: Noisy observation (매 iteration마다 노이즈)
[theta_j, ~] = solver_func(x_d_true, theta0, max_iter, rho, sigma);
% solver 내부에서 ik_loss_noisy()가 매번 노이즈 추가
```

## 예상 결과

### Deterministic (시나리오 0)
- 모든 알고리즘이 비슷한 성능

### Noisy Observation (시나리오 1) ⭐
- **Built-in**: 노이즈에 취약할 것으로 예상
  - 내부적으로 gradient-based method 사용
  - Noisy gradient로 인해 불안정
- **Stochastic methods**: 더 robust할 것으로 예상
  - 여러 샘플의 평균 효과
  - 노이즈에 덜 민감한 탐색

### 모델/제어 불확실성 (시나리오 2, 3)
- 최적화 과정은 깨끗하지만, 최종 평가에서만 노이즈
- 알고리즘 간 차이는 작을 것으로 예상

## 핵심 차이점

| 항목 | 이전 (Noisy Target) | 현재 (Noisy Observation) |
|------|---------------------|--------------------------|
| 노이즈 추가 시점 | 최적화 전 1회 | 매 iteration마다 |
| Loss function | Deterministic | Stochastic |
| 문제 성격 | Shifted target | True noisy observation |
| Gradient | Clean | Noisy (realistic!) |
| 알고리즘 차이 | 거의 없음 | 명확히 드러남 |

## 사용법

```matlab
% Deterministic 환경
[theta, ~] = localized_random_search(x_d, theta0, 1000, 0.005, 0);

% Noisy observation 환경 (sigma = 0.005)
[theta, ~] = localized_random_search(x_d, theta0, 1000, 0.005, 0.005);
```

## 참고사항

- Built-in solver는 noisy observation에 대해 완벽한 비교 불가
  - 내부 loss function을 변경할 수 없음
  - Wrapper는 "여러 번 시도"로 근사 (유리한 조건)
- Stochastic optimizer들은 이제 진짜 확률적 최적화를 수행
- 시나리오 1에서 가장 큰 차이가 나타날 것으로 예상

