import os
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.abspath(os.path.join(CURRENT_DIR, "fix_body.usd"))

ROBINION_CONFIG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,  # Gunakan file yang Anda upload
        #activate_armature=True,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=12, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.65), # Munculkan di ketinggian 1 meter agar tidak amblas
        #joint_pos={".*": 0.0}, # Posisi awal semua joint
        joint_pos={
            ".*shoulder_pitch.*": 0.174533,
            ".*l_shoulder_roll.*":-1.22173,
            ".*r_shoulder_roll.*": 1.22173,
            ".*l_elbow_yaw.*":-1.0472,
            ".*r_elbow_yaw.*":1.0472,
            ".*r_hip_pitch.*":0.349066,
            ".*l_hip_pitch.*":-0.349066,
            ".*r_ankle_pitch.*":-0.436332,
            ".*l_ankle_pitch.*":0.436332,
            ".*(head|torso|hip_yaw|hip_roll|ankle_roll).*":0.00,
        },
        #joint_vel={".*": 0.0},
    ),
    actuators={
        "body": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness={
                ".*head_yaw.*": 4.87521,
                ".*head_pitch.*": 0.80404,
                ".*l_shoulder_pitch.*": 14.78546,
                ".*r_shoulder_pitch.*": 15.49479,
                ".*shoulder_roll.*":15.42846,
                ".*elbow_pitch.*":19.46672,
                ".*elbow_yaw.*":7.01022,
                ".*torso.*":41.65867,
                ".*l_hip_yaw.*":103.82441,
                ".*r_hip_yaw.*":102.47318,
                ".*l_hip_roll.*":97.66118,
                ".*r_hip_roll.*":95.75182,
                ".*_hip_pitch.*": 180.0,  
                ".*_ankle_pitch.*": 140.0,  
                ".*ankle_roll.*": 90.0,
                ".*knee.*":0.0,
            },
            damping={
                ".*head_yaw.*": 0.00195,
                ".*head_pitch.*": 0.00032,
                ".*l_shoulder_pitch.*": 0.00591,
                ".*r_shoulder_pitch.*": 0.0062,
                ".*shoulder_roll.*":0.00617,
                ".*elbow_pitch.*":0.00779,
                ".*elbow_yaw.*":0.0028,
                ".*torso.*":0.5525,
                ".*l_hip_yaw.*":0.04153,
                ".*r_hip_yaw.*":0.04099,
                ".*l_hip_roll.*":0.03906,
                ".*r_hip_roll.*":0.0383,
                ".*_hip_pitch.*": 4.5,       
                ".*_ankle_pitch.*": 3.5,   
                ".*ankle_roll.*": 2.0,    
                ".*knee.*":0.0,
            },
        ),
    },
)
