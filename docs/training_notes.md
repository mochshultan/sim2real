# Training kinematics and reward notes

### 1. True Foot Tip Clearance Kinematics
The URDF `tibia_link` origin frame resides at the proximal knee joint ($Z \approx 0.44\text{ m}$). Measuring clearance from the link origin fails to reflect physical ground clearance at the foot pad.

The distal contact point is computed from the lowest mesh vertex in `Fr_tibia_pitch.STL`:
$$\mathbf{p}_{\text{tip\_local}} = [+\mathbf{0.087\text{ m}}, \mathbf{0.0\text{ m}}, -\mathbf{0.1634\text{ m}}]$$

The reward function `foot_clearance_dreamwaq` maps this offset to the world frame using the tibia link rotation matrix $\mathbf{R}_{\text{tibia}}(q)$:
$$\mathbf{p}_{\text{foot\_tip, world}} = \mathbf{p}_{\text{tibia, world}} + \mathbf{R}_{\text{tibia}}(\mathbf{p}_{\text{tip\_local}})$$

This formulation enforces physical foot clearance during the swing phase.

### 2. Base Height Ground Clearance
When the chassis collapses to the floor ($\approx 10\text{ cm}$), the entire shank rests horizontally. Contact sensors trigger across the shank surface, creating a false stance height estimation ($\approx 23\text{ cm}$).

The reward `base_height_l2_safe` calculates true vertical clearance between the base origin and the ground plane ($Z_{\text{root}} - Z_{\text{ground}}$), anchoring stance height at **0.21 m**.

### 3. Stand-Still Joint Deviation Penalty
When planar velocity commands drop below threshold ($\|\mathbf{v}_{\text{cmd}}\| < 0.1\text{ m/s}$), the `stand_still` reward penalizes joint deviations from nominal stance $q_0$ (front hip/knee `-1.51/1.22 rad`, rear hip/knee `-1.49/1.20 rad`, rolls `0.0 rad`). This prevents standing drift and limit-cycle oscillations while stationary.

### 4. Unitree A1-Style Analytical Spherical Foot Contact Modeling
Previously, NXP Jaguar modeled foot ground interaction using the convex-hull approximation of the entire tibia CAD mesh (`physics:approximation = "convexHull"`). In high-frequency physics engines (PhysX), polygonal mesh contacts suffer from:
1. **Discontinuous Normal Vectors & Force Spikes**: As the shank rotates during stance, the contact point abruptly jumps between mesh facets and vertices ("popping"), causing high impulsive torque chatter in the actuators.
2. **Erratic Contact Sensor Triggers**: Polygonal chatter creates high-frequency false positives/negatives in PhysX `contact_forces`, corrupting the `feet_air_time` reward and stance phase estimation.
3. **High Computational Overhead**: Iterative GJK/EPA mesh collision detection is computationally heavier and susceptible to numerical penetration compared to analytical shapes.

#### Architecture Adopted from Unitree A1 (`a1.urdf`):
Following the industry-standard quadruped design (Unitree A1/Go1/Go2, ANYmal, Boston Dynamics Spot), contact modeling is decoupled into:
- **Shank Bone Collision**: A slender primitive box (`<box size="0.02 0.02 0.12"/>`) along the tibia bone. This prevents the shank from clipping through obstacles while maintaining a clearance of $> 4.3\text{ cm}$ above the ground plane during normal gait.
- **Dedicated Foot Sphere Contact**: An explicit rigid link attached via a fixed joint (`dont_collapse="true"`) at the distal foot pad, with collision geometry defined as a **pure primitive sphere of radius $R = 0.01894\text{ m}$ (18.94 mm radius, 37.88 mm diameter)**:
  - **FR / BR**: $\mathbf{p}_{\text{foot}} = [+0.1035\text{ m}, -0.0175\text{ m}, -0.1445\text{ m}]$, with sphere bottom patch at $Z = -0.16344\text{ m}$ (exactly $0.0\text{ mm}$ gap, perfectly flush with the metal shank).
  - **FL / BL**: $\mathbf{p}_{\text{foot}} = [+0.1035\text{ m}, +0.0175\text{ m}, -0.1445\text{ m}]$, with sphere bottom patch at $Z = -0.16344\text{ m}$ (exactly $0.0\text{ mm}$ gap, perfectly flush with the metal shank).
  - **Flush Tangency (Gap = 0 mm)**: Protrusion is reduced to exactly $0.0\text{ mm}$ ($0.0000\text{ m}$), making the lowest tangent point of the analytical sphere contact surface coincide perfectly with the flat bottom face of the tibia foot pad ($Z = -0.16344\text{ m}$), delivering seamless continuous contact kinematics with zero artificial height bulge.
  - Simulator contact geometry belongs to the external training workspace. This deployment repository does not include a MuJoCo simulator or an Isaac Lab USD asset.

#### Physical Advantages:
- **Smooth Rolling Contact**: Exact analytical sphere normals ($\mathbf{n} = (\mathbf{x}_{\text{sphere}} - \mathbf{x}_{\text{ground}}) / R$) eliminate contact chatter across arbitrary foot roll and pitch angles.
- **Clean Contact Force Sensing**: Contact sensors target `SceneEntityCfg("contact_forces", body_names=".*_foot")` in IsaacLab and `geom name="*_foot"` in MuJoCo, delivering clean, continuous normal force profiles for gait phase detection and `feet_air_time` rewards.
- **Zero Height Bias**: Gap of $0.0\text{ mm}$ aligns analytical ground contact height directly with the CAD tibia frame, avoiding any height offsets between raw kinematics and simulation contact solvers.
- **$O(1)$ Solver Execution**: Analytical sphere-to-plane collision tests drastically increase simulation throughput and numerical stability in both PhysX and MuJoCo.

### 5. Velocity Tracking Error L2 Penalties (Anti-Stall, Anti-Stuck & Yaw Drift Control)
In standard exponential velocity tracking rewards ($r = w \exp(-\|\mathbf{v}_{\text{cmd}} - \mathbf{v}\|^2 / \sigma^2)$), when the robot encounters an obstacle or steep incline and becomes stuck ($\mathbf{v} \approx \mathbf{0}$), or refuses to turn ($\omega_z \approx 0$), the reward merely drops to near zero ($\approx 0.02$). Without an explicit penalty, the policy can find a degenerate passive local optimum: resting motionlessly against obstacles or resisting steering commands to avoid joint power and acceleration penalties.

#### a. Linear Velocity Tracking Error L2 Penalty (`track_lin_vel_xy_l2`):
Introduces an explicit quadratic penalty on linear velocity tracking error in the robot base frame:
$$r_{\text{track\_lin\_l2}} = -w_{\text{lin\_l2}} \cdot \sum_{i \in \{x, y\}} (v_{\text{cmd}, i} - v_{\text{actual}, i})^2$$
With $w_{\text{lin\_l2}} = -1.0$, a stalled robot commanded at $1.0\text{ m/s}$ incurs a continuous negative penalty of $-1.0$ per timestep (accumulating $-1000$ per episode). This creates an unyielding gradient compelling the policy to step actively, overcome obstacles, and never remain passively stuck.

#### b. Angular Velocity (Yaw) Tracking Error L2 Penalty (`track_ang_vel_z_l2`):
Introduces an explicit quadratic penalty on angular velocity $Z$ (yaw) tracking error in the robot base frame:
$$r_{\text{track\_ang\_l2}} = -w_{\text{ang\_l2}} \cdot (\omega_{\text{cmd}, z} - \omega_{\text{actual}, z})^2$$
With $w_{\text{ang\_l2}} = -0.5$, this term penalizes:
1. **Yaw Stall**: Resisting turning commands or failing to pivot around $Z$ when steering is requested.
2. **Uncommanded Yaw Drift**: Spinning out, twisting, or drifting off heading when commanded to walk straight or stand still ($\omega_{\text{cmd}, z} = 0$).
Together, these terms maintain precise heading and drive execution across challenging rough terrains and obstacles.
