repo: crop-row following with TD-MPC2 (model-based RL) in Gazebo/ROS Noetic, inside Docker.

runtime flow: train.py spawns roscore+roslaunch → Gazebo loads farm.world + robot → ROS nodes publish observations → GazeboEnv feeds them to TDMPC2 agent → agent publishes cmd_vel back.

dirs:
tdmpc2/          RL agent, training loop, env wrapper — read SKILLS/rl.md
src/             ROS packages (robot URDF, launch files, world, utility nodes) — read SKILLS/ros.md
docker/          Dockerfile, conda env, entrypoint — read SKILLS/docker.md
SKILLS/integration.md   full ROS↔RL topic contract with diagrams
SKILLS/          this folder — agent orientation files

key files one-liners:
tdmpc2/train.py          entry point; class OnlineTrainer owns the loop
tdmpc2/config.yaml       all hyperparams + robot_ns + launchfile path
tdmpc2/envs/gazebo.py    GazeboEnv: wraps ROS into a gym.Env
tdmpc2/tdmpc2.py         TDMPC2 agent: act() and update()
tdmpc2/common/buffer.py  replay buffer
src/terra_gazebo/launch/terra_gazebo.launch   launches world + robot + utility nodes
src/robot_description/urdf/robot.urdf.xacro  robot model (swap to change robot)
src/terra_utils/scripts/rowfollow_gt.py       publishes distance_error from GT pose
src/terra_utils/scripts/perception_stub.py   publishes zero heatmap (replace with real perception)
src/terra_worlds/worlds/farm.world            Gazebo scenario
docker/ROS_noetic.dockerfile                  image definition
docker/environment.yaml                       conda/pip packages

submodules: none. all src/ packages are inline regular files.
