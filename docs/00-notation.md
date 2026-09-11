# 00: Notation, frames and conventions

## Purpose

Fix the conventions every other document and every line of code uses: which frames exist,
what a transform symbol means, how a pose is perturbed, how sensor noise is written, and how
to check an axis convention against real numbers instead of trusting a figure.

## Derivation

### Frames

| Symbol | Frame | Definition |
|---|---|---|
| $W$ | world | The ground-truth frame of the survey scanner. Fixed. Assumed $z$ up until checked against the ground-truth $z$ range of a sequence with a known descent (exp14 goes down a staircase). |
| $I$ | IMU | The Bosch BMI085 inside the Alphasense. The dataset ground truth is the pose of $I$ in $W$. |
| $L$ | LiDAR | The Hesai PandarXT-32 frame. Scans are measured here. |
| $L_k$ | LiDAR at scan $k$ | The $L$ frame at the start time $t_k$ of scan $k$. Registration estimates $T_{W L_k}$ or, in the odometry front end, $T_{L_{k-1} L_k}$. |
| $K_j$ | keyframe $j$ | A subset of the $L_k$ frames the back end keeps as graph nodes. |

### Rigid transforms

A rigid transform is a rotation matrix $R \in SO(3)$ and a translation $t \in \mathbb{R}^3$:

```math
T_{AB} = \begin{bmatrix} R_{AB} & t_{AB} \\ 0 & 1 \end{bmatrix},
\qquad
p_A = R_{AB}\, p_B + t_{AB}.
```

$T_{AB}$ maps points expressed in $B$ to points expressed in $A$. Equivalently it is the pose
of frame $B$ seen from $A$: $t_{AB}$ is the origin of $B$ expressed in $A$ and the columns of
$R_{AB}$ are the axes of $B$ expressed in $A$.

The subscripts chain right to left and the inner index cancels:

```math
T_{AC} = T_{AB}\, T_{BC},
\qquad
T_{BA} = T_{AB}^{-1} = \begin{bmatrix} R_{AB}^\top & -R_{AB}^\top t_{AB} \\ 0 & 1 \end{bmatrix}.
```

In code the same symbol is written `T_A_B`, so `T_W_L = T_W_I * T_I_L`. The calibration
file gives the LiDAR with `parent: imu`, which is $T_{IL}$: it maps LiDAR points into the IMU
frame, and its translation is the LiDAR origin expressed in $I$.

### Quaternions

Unit quaternions are Hamilton convention and are **stored and printed as $(x, y, z, w)$**,
the order used by ROS messages, the Hilti calibration yaml, TUM trajectory files, `scipy`
and evo. Two traps:

- `Eigen::Quaterniond(w, x, y, z)` takes the scalar first in its constructor although
  `coeffs()` returns $(x, y, z, w)$. Every construction site in the code goes through
  `SE3::fromQuaternionXYZW` so this is decided once.
- Reading the calibration quaternion in the wrong order does not produce garbage. It produces
  a different, valid rotation and a plausible-looking wrong trajectory. The worked example
  below and the assertions in `scripts/gt_to_tum.py` exist to catch exactly that.

### Angular velocity and the gyro

$\omega_B$ is the angular velocity of frame $B$ relative to the inertial frame, expressed in
$B$. The gyro inside the IMU measures $\omega_I$. For a rotation $R_{WB}(t)$,

```math
\dot R_{WB} = R_{WB}\, [\omega_B]_\times ,
```

with $[\cdot]_\times$ the skew-symmetric matrix of doc 01. This is the equation doc 02
integrates.

### Perturbations: $\boxplus$ and $\boxminus$

Optimisers work in a local vector space around the current estimate. All perturbations in
this project are **right (body-frame) perturbations**:

```math
R \boxplus \delta = R\, \mathrm{Exp}(\delta), \qquad
R_1 \boxminus R_2 = \mathrm{Log}(R_2^{-1} R_1), \qquad \delta \in \mathbb{R}^3,
```

```math
T \boxplus \xi = T\, \mathrm{Exp}(\xi), \qquad
T_1 \boxminus T_2 = \mathrm{Log}(T_2^{-1} T_1), \qquad
\xi = \begin{bmatrix} \theta \\ \rho \end{bmatrix} \in \mathbb{R}^6 .
```

The 6-vector is **rotation first, translation second**, $\xi = (\theta, \rho)$. This matches
GTSAM's `Pose3` tangent ordering and small_gicp's 6×6 Hessian, so covariances move between
the three without permutation. Exp and Log are defined in doc 01.

### Noise

A gyro sample at time $t$ is modelled as

```math
\tilde\omega = \omega + b_g + n_g, \qquad
n_g \sim \mathcal N(0, \sigma_g^2 I), \qquad
\dot b_g = n_{b_g}, \quad n_{b_g} \sim \mathcal N(0, \sigma_{b_g}^2 I),
```

and the accelerometer likewise with $b_a$, $n_a$, $\sigma_a$, $\sigma_{b_a}$. $\sigma_g$ is a
continuous-time noise density in $\mathrm{rad\,s^{-1}/\sqrt{Hz}}$; a sample at rate
$1/\Delta t$ has discrete standard deviation $\sigma_g / \sqrt{\Delta t}$. The dataset's
`imu_noise_calibration.bag` provides Allan-variance inputs; the numbers actually used are
recorded in `configs/hilti22.yaml` with their provenance.

Covariance matrices are $\Sigma$, information (precision) matrices are $\Lambda = \Sigma^{-1}$.
A measurement $z$ with model $h(x)$ and covariance $\Sigma$ contributes the residual
$r = z \boxminus h(x)$ and the cost $\tfrac12 r^\top \Lambda r$.

## Worked example: which way is up in frame $I$?

Three sources describe the axes of $I$ and they do not obviously agree. Resolving this is the
template for every convention check in the project: derive what each source implies, then
measure.

**Source 1, the calibration yaml.** `lidar_calibration.yaml` contains, under the IMU
intrinsics, `gravity: [0.0117, 9.8050, -0.0087]`, a vector of magnitude $9.805\ \mathrm{m/s^2}$
along $+y_I$. An accelerometer at rest measures the specific force $f = a - g = -g$, which
points **up**. So if this vector is the mean accelerometer reading, $+y_I$ points up; if it is
the gravity vector itself, $+y_I$ points down. The file does not say which.

**Source 2, the extrinsic quaternion.** $T_{IL}$ has quaternion $(x,y,z,w) = (0.7071068,
-0.7071068, 0, 0)$: a rotation by $\pi$ about the unit axis $a = (1, -1, 0)/\sqrt2$. A
rotation by $\pi$ about $a$ maps $v \mapsto 2a(a^\top v) - v$, so

```math
R_{IL}\, x_L = -y_I, \qquad R_{IL}\, y_L = -x_I, \qquad R_{IL}\, z_L = -z_I .
```

The LiDAR spin axis $z_L$ is therefore antiparallel to $z_I$. On the handheld device in the
paper's Figure 1 the LiDAR sits below the cameras with its spin axis vertical, which makes
$z_I$ the vertical axis, contradicting Source 1's $y$.

If the quaternion were misread as $(w,x,y,z)$ it would be a rotation by $\pi/2$ about $-x$,
mapping $z_L \mapsto +y_I$, which would make Source 1 and Source 2 agree for the wrong
reason. This is why the known-answer assertion in `gt_to_tum.py` checks
$R_{IL} z_L = -z_I$ specifically.

**Source 3, measurement.** `scripts/inspect_bag.py` prints the mean accelerometer vector over
the first second of exp14, during which the operator is standing still. Its dominant axis and
sign settle the question: the axis with magnitude $\approx 9.8$ is the vertical of $I$, and a
positive sign means that axis points up. The likely reconciliation is that the calibration
bag was recorded with the device lying on its side, so Source 1 describes a different
attitude, not a different frame. Whatever the outcome, the measured value is what goes into
`configs/hilti22.yaml`, with the reasoning above as the comment.

**Result (exp14, first 1.0 s, 401 samples, `scripts/inspect_bag.py`, 2026-09-11).** The mean
specific force is $(0.173,\ 0.164,\ -9.660)\ \mathrm{m/s^2}$ with $|f| = 9.663$ and a per-sample
standard deviation of $0.015$. The dominant axis is $z_I$ with a **negative** sign, so
$-z_I$ points up and $z_I$ points down. Through Source 2, $z_L = -z_I$ then points up: the
LiDAR spin axis is vertical, as the figure shows. Sources 2 and 3 agree, and Source 1 is the
resting reading in a different attitude (device on its side during the 90-minute IMU
calibration), not a statement about which axis is vertical in normal use. The gyro over the
same window has mean $(1.06, -0.28, -0.04)\times 10^{-3}$ rad/s and per-sample standard
deviation $(2.8, 3.8, 3.2)\times 10^{-3}$ rad/s at 399.2 Hz; these feed doc 02. The magnitude
$9.66$ rather than $9.81$ is left as an open question for doc 06 (accelerometer scale or
residual motion); it does not affect the axis conclusion.

Consequences: the up direction in $L$ is $R_{IL}^\top(-z_I) = +z_L$, the IMU gyro axes map
into $L$ through $R_{IL}^\top$ (doc 02), and the world $z$ range of the ground truth must
decrease through the exp14 staircase (it spans 4.18 m in $z$; sign checked in week 2 when
the trajectory is plotted).

## Spec

### Signatures

No code is generated from this document. It constrains the others:

- Every transform variable is named `T_A_B` and satisfies `p_A = T_A_B * p_B`.
- Every quaternion crossing an API boundary is $(x,y,z,w)$; construction goes through
  `SE3::fromQuaternionXYZW(qx, qy, qz, qw, t)`.
- Every 6-vector tangent is $(\theta, \rho)$, rotation first.
- Every perturbation is a right perturbation, `T * SE3::Exp(xi)`.

### Invariants

- `T_A_B * T_B_C` has the inner index cancelling; a mismatch is a naming bug.
- $R_{IL} z_L = -z_I$ and $R_{IL} x_L = -y_I$ for the Hilti extrinsic.
- The measured resting accelerometer norm is within 2 % of $9.81\ \mathrm{m/s^2}$ (measured 9.66; see the worked example) and its dominant axis is $-z_I$.

### Tests

- `scripts/gt_to_tum.py`: `check_extrinsics` asserts the two axis mappings above, the
  inverse round trip $T_{LW}(T_{WL}\,p) = p$ to $10^{-9}$, and that the LiDAR origin in the
  world equals $R_{WI} t_{IL} + t_{WI}$.
- `scripts/inspect_bag.py`: prints the resting accelerometer mean and its dominant axis; the
  result is recorded in `configs/hilti22.yaml` and quoted in doc 02.
