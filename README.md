# Robinion Locomotion User Guide
This guide provides the steps to set up the workspace, create a virtual environment, and test Reinforcement Learning (RL) simulation for our Robinion humanoid robot using **NVIDIA Isaac Lab**.

---

## Setup Main Environment

The Robinion locomotion framework was developed using Python 3 within the NVIDIA Isaac Sim and Isaac Lab ecosystems. We prefer a **Conda virtual environment** to manage all system dependencies. Please follow the guide below to set up your environment.

### 1. Install IsaacSim

To install Isaac Sim, you can follow the official tutorial documentation from the link below (it is highly recommended to use version 5.1.0):

* [NVIDIA Isaac Sim Installation Guide](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html)

### 2. Install IsaacLab

To install Isaac Lab, you can follow the official tutorial documentation from the link below:

* [Isaac Lab Installation Guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)

### 3. Verifying the Isaac Lab installation
To verify that the installation was successful, run the following command from the top of the repository:

    ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --viz kit
The above command should launch the simulator and display a window with a black viewport. You can exit the script by pressing Ctrl+C on your terminal.

### 4. Testing Default Example
Since our project is built upon the default H1 humanoid robot template provided directly by Isaac Lab, we will first run a test training session using Isaac Lab's built-in H1 model.

This step serves as an initial environment test to verify that your GPU setup, RL pipeline, and dependencies are working properly before we integrate the custom Robinion model and configurations.

To test the default training workflow, run:

    ./isaaclab.sh -p train-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-H1-v0

## Setup Project Workspace

Since this project builds upon the native H1 locomotion framework by swapping the robot asset and adjusting its configuration parameters, you only need to integrate the custom files into the default Isaac Lab directory tree:
### 1. Clone the custom Robinion project:
Clone the repository into your home directory:
   
    git clone git@github.com:choirulprayoga/Robinion_Parallel_IsaacLab.git ~/Robinion_Project

### 2. Integrate Custom Assets and Configurations
  Copy the files from your cloned repository directly into the native Isaac Lab task directories
   
    # Copy the Robinion robot USD assets into Isaac Lab's locomotion config directory
    cp -r ~/Robinion_Project/assets/* ~/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/
    
    # Copy the custom environment configuration files into Isaac Lab's velocity task directory
    cp -r ~/Robinion_Project/envs/* ~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/h1

### 3. Train with the Robinion Model
Now that the Robinion assets and custom environment configurations have been successfully integrated into your Isaac Lab installation, you can launch the RL training process using the custom task:

    ./isaaclab.sh -p train-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-H1-v0 

To run the simulation without rendering the UI (Headless mode for faster training):

    ./isaaclab.sh -p train-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-H1-v0 --headless

To adjust the number of parallel environment instances:
    
    ./isaaclab.sh -p train-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-H1-v0 --num_envs=10
