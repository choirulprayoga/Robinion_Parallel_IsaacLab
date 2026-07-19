import argparse
from isaaclab.app import AppLauncher

# 1. Setup App Launcher
parser = argparse.ArgumentParser(description="Script untuk memunculkan robot Robinion di Isaac Lab.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. Import library Isaac Lab setelah app diluncurkan
import torch
import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from robinion_cfg import ROBINION_CONFIG # Perbaikan: Sesuaikan dengan nama file robinion.py

from isaaclab.assets import AssetBaseCfg 

def main():
    # Konfigurasi Simulasi
    sim_cfg = sim_utils.SimulationCfg(device="cuda:0")
    sim = sim_utils.SimulationContext(sim_cfg)
    
    # Set tampilan kamera agar langsung melihat ke robot
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.5])

    # 3. Mendesain Scene (Lingkungan)
    scene_cfg = InteractiveSceneCfg(num_envs=1, env_spacing=5.0)
    
    # Tambahkan Ground Plane (Lantai)
    scene_cfg.ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    
    # Tambahkan Cahaya (Distant Light)
    scene_cfg.light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    # 4. Tambahkan Robot Humanoid Anda
    scene_cfg.robot = ROBINION_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Inisialisasi Scene
    scene = InteractiveScene(scene_cfg)

    # Reset simulasi untuk menerapkan posisi awal fisik
    sim.reset()
    
    # Ambil referensi objek robot dari scene untuk dikontrol
    robot_asset = scene["robot"]
    
    print("[INFO]: Robot Robinion berhasil dimunculkan dengan kontrol aktif!")

    # 5. Main Loop (Simulasi berjalan)
    while simulation_app.is_running():
        
        # --- TAMBAHKAN KONTROL DI SINI ---
        # Buat tensor target posisi untuk semua joint (misalnya target = 0.0 radian untuk semua joint)
        # Sesuai dengan ukuran jumlah joint terkontrol pada robot asli Anda
        #targets = torch.zeros(robot_asset.data.joint_pos.shape, device=sim.device)
        targets = robot_asset.data.default_joint_pos.clone()
        
        # Kirim target posisi ke aktuator/motor robot
        robot_asset.set_joint_position_target(targets)
        
        # Terapkan perintah kontrol tersebut ke simulator fisik
        robot_asset.write_data_to_sim()
        # ---------------------------------

        # Update data sensor dan posisi dari simulator ke Isaac Lab
        scene.update(dt=sim.get_physics_dt())
        
        # Step simulasi fisika
        sim.step()

if __name__ == "__main__":
    main()
    simulation_app.close()
