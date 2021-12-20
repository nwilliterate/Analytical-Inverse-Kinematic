

# Practical analytical inverse kinematic approach for 7-DOF space manipulators with joint and attitude limits



### 논문의 초록

-----

​	관절과 자세 제한을 가진 7 자유도를 가지는 매니퓰레이터에 대한 **분석학적 역기구학 솔루션(the analytical inverse kinematic solution)**을 계산하기 위한 접근법을 제안한 논문이다. 

​	기존의 속도로 자코비안 행렬을 계산하여 관절의 범위를 제한하는 속도 기반의 접는법(the velocity-level Jacobian matrix)을 사용하나, 본 논문에서는 실행가능한 역기구학 솔루션의 범위를 평가하여 **위치 기반의 접근법(a position-based approach)**을 제시한다. 

​	최적의 솔루션을 얻기 위해,  매니퓰레이터의 작용하는 교란(disturbance)을 기반으로 추정 기반으로 한다.

- 매니퓰레이터의 여유자유도(the redundancy) 개념을 정의하고, 각각의 관절은 여유자유도(the redundancy)에 의해 매개변수화 된다.

- 관절의 제한 범위가 여유자유도(the redundancy)에 미치는 영향에 대해서 논의한다.
- 7-DOF 조작기의 역기구학적 문제를 다루기 위해 실용적인 접근법을 제안하고 있다.
- 마지막으로, 이 접근법의 유효성은 수치학적 시뮬레이션으로 검증된다.

```
- 여유자유도(redundancy)란 매니퓰레이터의 자코비안 행렬에서 열의 수가 행의 수보다 많을 때를 말하며, 무한한 역기구학 해가 존재하게 됩니다.즉, 여유자유도는 주어진 작업 공간 자유도(defgree of manipulability, DOM) 이상의 관절 공간 자유도를 보유한 경우를 의미하며, 이 경우 주어진 작업을 수행하고 남는 자유도를 활용하여 추가적인 작업(관절 공간 제한 회피, 특이점 회피, 충돌 회피)이 가능하게 됩니다. 예를들어 6축 로봇의 경우 3차원 공간 상에 위치와 방향각을 모두 조절하는데, 이 때 여유자유도를 가지지 않은다고 할수 있으며, n+6개의 관절을 가지게 되는 경우 3차원 공간 상에 위치와 방향각을 모두 조절하고도 n개의 여유자유도를 가진다고 말할 수 있다.
```



### 서론

-------------------

##### 여유자유도 가지는 매니퓰레이터에 대한 역기구학 솔루션이 어려운 점.

- 일반적으로, 여유자유도를 가지는 매니퓰레이터의 분석학적 역기구학 솔루션(an analytical inverse kinematic solution)는 엔드이펙터가 변경되지 않는 자세로 이미 고정되어 있어 솔루션을 결정할 수 없기 때문에 어려운 문제이다.
- 우주분야에서 매니퓰레이터의 자세 및 위치는 매니퓰레이터의 행동에 의해 영향을 받을 수 있다.

##### 기존의 관절과 자세를 가지는 역기구학 해결책

- 기존의 관절과 자세를 가진 조작기의 제어 문제에 대한 해결책은 속도-수준(velocity-level)의 접근법으로 제한되어 옴. 하지만 속도-수준(velocity-level)는 여유자유도 매니퓰레이터의 **역기구학의 다중-솔루션(multi-solution)의 특징**을 거의 고려하지 않는다. [2, 3]
- 따라서 분석학적 역기구학 접근법이 필요하나, 이미 일부 연구자들 연구를 진행하였다.
- [4]에서는 DOF 매니퓰레이터를 위한 폐쇄형 inverse solution을 계산하는 방법을 제안함. 하지만, **매개 변수 선택은 매우 어렵고 조인트 한계**가 고려되지 않음.
- [5, 6]은 어깨와 손목의 한계가 어떻게 inverse solution에 영향을 미치는지에 대해 논의 했지만, **매니퓰레이터의 self-motion은 무시**함.
- [7]은 휴머노이드 로봇 암에 대한 역기구학 알고리즘을 제안했지만, **관절의 한계에 분석하지 않음**.
- [8]는 관절의 한계를 다루기 위해 완벽한 방법을 도출 했으며, 이는 [9]에서 더 발전했음. 하지만, 이 방법은 **지상에서만 적용이 가능**하다. 
- 다양한 방법에 대한 요약은 [10 - 13]에 서술되어 있음.



##### 본 연구의 개선점

- [9]에서 보다 일반적인 경우(특히, 우주 환경)에서 개념적인 의미에서 지상과 완전히 다른 경우로 확장함.
- 특이성의 유형(The types of singularity)은 포괄적이고 명확하게 논의되는 반면, 실용적인 응용을 처리하기위한 유용한 프로세스가 제안함.

- 마지막으로이 알고리즘의 유효성을 검증하기 위해 특정 샘플이 아닌 좀 더 일반적인 샘플을 테스트함.



### 모델링 및 파라미터화

--------

#### 연구 대상의 모델링

- 일반적으로 여유자유도 매니퓰레이터에는 SRS (Sphere-Rotation-Sphere), RSS (Rotation-Sphere-Sphere), UUS-A (Universe-Universe-Sphere A type), UUS-B (Universe-Universe-Sphere B type)와 같은 네 가지 유형이 있다.

<img src="C:\Users\USER\AppData\Roaming\Typora\typora-user-images\image-20200716225808770.png" alt="image-20200716225808770" style="zoom:67%;" />



<img src="C:\Users\USER\AppData\Roaming\Typora\typora-user-images\image-20200716225844691.png" alt="image-20200716225844691"  />

```
Type A : SRS (Sphere-Rotation-Sphere)
Type B : UUS-A (Universe-Universe-Sphere A type) 
Type C : RSS (Rotation-Sphere-Sphere) 
Type D : UUS-B (Universe-Universe-Sphere B type)
Universe -> 경첩의 구성을 가짐/ 커플링된 조인트
"SINGULARITY ANALYSIS OF A KINEMATICALLY SIMPLE CLASS OF 7-JOINTED REVOLUTE MANIPULATORS"(2009)
```

- 그러나, **제안된 접근 방식은 SRS 구조에만 적용이 가능**하다. ***(논문의 단점)***

- [15]에서는 SRS 구조를 가지는  7-DOF 매니퓰레이터의 최적 모델임을 보여줌.

![figure1](https://media.springernature.com/lw685/springer-static/image/art%3A10.1007%2Fs11370-015-0180-3/MediaObjects/11370_2015_180_Fig1_HTML.gif)

- SRS 매니퓰레이터는 일반적으로 
  - the shoulder (joints 1, 2 and 3)
  - elbow (joint 4)
  - wrist (joints 5, 6, and 7),
  - upper arm (base-proximal link)
  -  and forearm (base-distal link) joints 으로 구성됨.
- 손목 관절은 어깨 관절과 동일한 특징을 가짐.



#### 파라미터화

- 7-DOF는 조작기의 여유자유도 구조 유형이므로, 여유자유도 DOF를 설명하는 데 사용할 수있는 매개 변수가 필요하다.
- 이것을 다루는 몇가지 연구를 소개
- [4]에서는 여유자유도를 가지는 매니퓰레이터를 나타내는 것을 제안함.
- [16]은 관절 1과 3이 여유자유도를 매개 변수화하기위한 최선의 선택으로 나타내는 것을 제안함.
- [17]에서는 Joint 7을 잉여 조인트로 사용했다. 하지만 7-DOF 매니퓰레이터는 Joint 사이에 비선형 관계가 있기 때문에, 다른 조인트를 기반으로 여유자유도 조인트 각도를 특별하게 결정할 수 없음.
- 이전의 연구 [15]는 또한 "self-motion" 개념을 다룸. 즉, 엔드 이펙터의 위치와 방향이 고정되어 있어도 팔과 팔뚝이 어깨와 손목 사이의 선을 중심으로 회전 할 수있다

![그림 2](https://media.springernature.com/lw685/springer-static/image/art%3A10.1007%2Fs11370-015-0180-3/MediaObjects/11370_2015_180_Fig2_HTML.gif)

- 기준면은 조인트 3가 0일 때(이 시점에서, 조작기는 6-DOF 매니퓰레이터와 동일하고 기준면은 고유하다), upper arm과 forearm에 의해 결정된다.
- 위에서 정의한 바와 같이, upper arm과 forearm의 평면은 어깨와 손목으로 고정 된 축을 중심으로 회전 할 수 있다. 회전 후 새 평면을 arm plane이라고하며, ψ는 두 평면 사이의 거리 각도 인 *arm angle*를 정의하는 데 사용됨. 
- 따라서 각 관절의 회전 행렬을 다음과 같은 형식으로 나타냄.

$$
^{b}R_{s} =\  ^{0}R_{1} \cdot \   ^{1}R_{2} \cdot \ ^{2}R_{3} \\
^{s}R_{e} =\  ^{3}R_{4} \\
^{e}R_{t} =\  ^{4}R_{5} \cdot \   ^{5}R_{6} \cdot \ ^{6}R_{7}
$$

- 이때의 회전 행렬

$$
^{i-1}R_{i} = 
\begin{bmatrix}
cos\theta_i & -sin\theta_i\cdot cos\alpha_{i-1} & sin\theta_i \cdot sin\alpha_{i-1} \\
sin\theta_i & cos\theta_i\cdot cos\alpha_{i-1} & -cos\theta_i \cdot sin\alpha_{i-1} \\
0 & sin\alpha_{i-1} & cos\alpha_{i-1} \\
\end{bmatrix}
$$

- 엔드 이펙터(Tip)의 위치와 방향은 다음과 같이 나타냄.

$$
^{b}x_{t} = \ ^{b}I_{bs} + \ ^{b}R_{s} \cdot [ \ ^{s}I_{se} + \ ^{s}R_{e} \cdot(\ ^{e}I_{ew} +\ ^{e}R_{ew} \cdot\ ^{w}I_{wt})] \\
^{b}R_{t} =\  ^{b}R_{s} \cdot \   ^{s}R_{e} \cdot \ ^{e}R_{t} \\
$$

- 엔드 이펙터의 자세가 고정되면 손목의 위치는 변하지 않습니다. 따라서 코사인 법칙을 사용하여 팔꿈치 각도를 계산할 수 있다.
- 분명히 팔꿈치는 관절 4만 포함하므로 다음과 같이 나타냄.

$$
\theta_e = \theta_4 = acos{\lVert ^{b}x_{sw} \lVert^2 - d_{se}^2 -d_{ew}^2 \over 2\ \cdot d_{se} \cdot d_{ew} }
$$

- 우리는 (2.3)에서 두 가지 솔루션을 얻을 수 있으나, 하나는 매니퓰레이터의 셀프-모션으로 인해 중복되며 양수 값만 필요하다.
- 

~(중략)~



### 관절의 제한 범위에 따른 역기구학 솔루션

-----------

- 매니퓰레이터 및 케이블의 구조 등 다양한 이유로 각 조인트는 작업장 가장자리에 닿지 않는 범위를 충족시켜야한다. 

- 여기서 ψ가 θ의 유일한 매개 변수라는 것을 이미 알고 있으며, 관절의 한계가 ψ에 미치는 영향을 분석한다.

- 식 (2.4)와 (2.5)에서 관절이 회전 할 때 θ와 ψ 사이에는 두 가지 유형의 관계, 즉 Tan 유형과 Cos 유형이 있음을 알 수 있다.

- θ와 ψ 사이에는 관계
  - Monotonic 1
  - Monotonic 2
  - Tan singular 1
  - Tan singular 2
  - Tan cyclic 1
  - Cos cyclic 1
  - Cos singular 1
  - Cos singular 2
  
- 실제로 조인트 5,6,7에 대한 방정식의 프로파일은 모두 이 특성을 공유하지만 조인트 1, 2, 3에 대한 방정식의 프로파일에는 이 기능이 없다. 

- 다음과 같이 대칭 축을 간단히 결정할 수 있습니다.

- 먼저 엔드 이펙터의 방향을 회전 행렬에서 Z–Y–Z Eulerian 각도로 변환한 다음 기본 좌표계의 z축 기준으로 y축의 호를 계산한다.

- ψ와 θ1, θ2, θ3, θ5, θ6, θ7의 관계는 Monotonic (tan 유형만 포함), cyclic (tan 유형과 cos 유형 포함) 및 singular 유형 (tan 유형과 cos 유형 포함), 각 유형은 팔 각도 값에 영향을 미친다.

  - **단조로운 유형(Monotonic type)** : θ와 ψ 사이의 *일대일 대응(one-to-one correspondence)*이다. 따라서 각각의 고유한 $\psi^u$와 $\psi^l$을 얻을 수 있으며, 이는 상한과 하한 범위를 뜻한다. $\psi = [\psi^l , \psi^u ]$

  - **순환하는 유형(Cyclic type)** : $\theta^u_i > \theta^{\text{max}}_i$ 이나  $\theta^l_i < \theta^{\text{min}}_i$ 인 경우,  $\psi$ 의 값은 2개만 존재하므로, $\theta$ 값이 극한을 달성하도록 보장한다. 하지만 $\theta^u_i < \theta^{\text{max}}_i$ 이나  $\theta^l_i > \theta^{\text{min}}_i$인 경우, 두개의 다른 값은 $\psi^u$ 또는 $\psi^l$에 대응된다. 일반적으로 우리는 위의 모든 범위의 교집합에서 연속 또는 별도의 범위를 얻을 수 있다. 이 범위는 $\psi$의 실현 가능한 값의 집합이며, 각 관절의 경계를 만족시킨다.

  - **단일 유형(Singular type)** : 목표를 달성하기 위해서는, 모든 유형의 특이성을 논의해야 하며, : 기구학적 특이점(kinematic singularity)[18], 알고리즘 특이점(algorithm singularity), 반특이성(semi-singularity)[19]의 세가지 유형으로 대략 나누어 분류한다.

    - 7축 매니퓰레이터의 **기구학 특이성(the kinematic singularity)**은 어깨(shoulder), 팔꿈치(elbow), 손목(wrist) 특이점으로 구성되며,  팔꿈치 및 손목 특이점은 매니퓰레이터의 영향을 미치지 않는다. 하지만 어깨 특이점이 발생하면, 조인트 1과 연장성에 있는 손목의 결과 때문에 조인트 1의 값을 결정할 수 없다.  이 경우에는 조인트 1에 0의 값만 지정할 수 있다.
    - **알고리즘 특이성(Algorithm singularity)** : 각 관절 각도는 θ1, θ3, θ5, θ7의 방정식을 암시적 함수 형태로 표현할 수 있다. 

    $$
    \tan\theta_i = {a_n\cdot\sin\psi+b_n\cdot\cos\psi+c_n\over a_d\cdot\sin\psi+b_d\cdot\cos\psi+c_d} \\
    \cos\theta_i=a\cdot\sin\psi+b\cdot\cos\psi+c
    $$

    - 시작 부분에서 $a^2+b^2-(c-1)^2=0$ 또는 $a^2+b^2-(c+1)^2=0$이고 cos유형과 tan 유형에 대한 $a_t^2+b_t^2-c_t^2=0$  이 조건을 확인했다. $a_t =b_dc_n-b_nc_d, b_t =a_nc_d-a_dc_n, c_t=a_nb_d-a_db_n.$
    - cos 유형의 경우 그림 3g에서 h는 ψ가 특이점에 도달하더라도 일반적으로 θi의 고유 한 값을 결정할 수 있다. θi의 값을 특이점에서 직접 계산할 수는 없지만 왼손 및 오른손 한계는이를 보장하는 데 도움이 될 수 있다[9]. 또한 cos 특이 형은 순환 유형과 유사한 기능을 공유한다.
    - 그러나, tan 단수형은 ψi가 단수형을 달성 할 수 있다는 점을 제외하고는 임의의 ψ에 대해 임의의 ψ에 대해 독특하게 결정될 수 있기 때문에 순환 형과 비교하여 약간 다른 특징을 갖는다. 따라서이 점을 ψ의 범위에서 분리해야한다.
    - SAS (Semi algorithm singleularity) : SAS는 일종의 알고리즘 특이성입니다. 조인트가 위치 제한에 도달하면 발생합니다. 운동학 특이점보다 해결하기가 더 어렵습니다. SAS의 문제를 피하기 위해 운동학 역 솔루션을 해결하기 위해 간단한 단항 최적화 알고리즘이 채택되었습니다. 그리고 솔루션은 모든 조인트 한계에서 멀어집니다.

우선, 각각의 관절 한계를 충족시키는 팔 각도의 값이 결정되며, 이는 전술 한 방법을 통해 이루어진다. 즉, 팔 각도 범위에서 교차점을 취하는 것입니다. 그러나, 팔 각도의 범위가 하나의 조인트 또는 그 이상의 한계에 접근 할 가능성이 여전히 존재한다. SAS가 발생할 수 있지만 예상하지는 않습니다. 이 논문에서, θ d i = (θ l i + θ u i) / 2는 모든 예상 조인트 값을 나타냅니다. 따라서, bRd s는 예상 어깨 관절의 회전 행렬을 나타내고, eRd t는 예상 손목 관절의 회전 행렬을 나타낸다. 실제 bR과 eRt는 최소 거리에있을 경우 예상 어깨와 손목 관절로 간주 될 수 있습니다. 대부분의 경우 조인트 한계에서 멀어집니다. 일반화 된 회전 행렬에 따르면, R이 실제 회전 행렬이고 Rd가 예상 회전 행렬 인 경우 R · RdT는 단위 행렬 I 3에 더 가깝습니다. 또한, 방향은 두 회전 행렬에 의해 훨씬 더 가깝게 표현됩니다



