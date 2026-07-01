catkin_make fails: fatal error: hector_gazebo_plugins/sensor_model.h
cause: ros-noetic-hector-gazebo-plugins not installed
fix: apt-get install ros-noetic-hector-gazebo-plugins (already in Dockerfile:44; only missing on old images)

Hydra error: Could not find 'hydra/launcher/submitit_local'
cause: old config.yaml had submitit_local override; hydra-submitit-launcher was removed from environment.yaml
fix: already fixed — tdmpc2/config.yaml defaults block has no launcher override; basic is default

train.py AssertionError at line 242: assert torch.cuda.is_available()
cause: no GPU / drivers not passed through to container
fix: verify nvidia-smi works on host and nvidia-container-runtime is installed; run.sh must pass --gpus all

Gazebo not ready / service timeout on first step
cause: time.sleep(8) at gazebo.py:108 is the startup wait; if Gazebo is slow (first launch loads models) 8s may not be enough
fix: increase RESET_DELAY or add retry logic in step(); do not reduce below 8s

cv_bridge ImportError or libffi mismatch
cause: conda libffi overrides system libffi7 that cv_bridge needs
fix: Dockerfile:67 reinstalls libffi7; if running outside Docker: apt-get install --reinstall libffi7 libffi-dev
also check PYTHONPATH is empty (not set to conda site-packages)

cmd_vel not moving robot
cause: robot URDF plugin must subscribe geometry_msgs/Twist, not TwistStamped
gazebo.py publishes Twist since the public release refactor; confirm with: rostopic echo /<ns>/cmd_vel
if using old terrasentia plugin: it expects TwistStamped — you need to either update the plugin or add a relay node

GAZEBO_MODEL_PATH not set / Gazebo can't find models
cause: terra_worlds models not on search path
fix: terra_gazebo.launch:L22 sets GAZEBO_MODEL_PATH to $(find terra_worlds)/models; also set after sourcing devel/setup.bash
if launching Gazebo manually without the launch file: export GAZEBO_MODEL_PATH=/ros_ws/lecropfollow/src/terra_worlds/models

device mismatch: RuntimeError tensor on cpu vs cuda in EMA step
cause: filtered_action initialized on cpu; robot_action may be on cuda
fix: already handled at gazebo.py:163 — self.filtered_action = self.filtered_action.to(robot_action.device)

perception_stub warning: agent observes all-zero heatmap
this is expected behavior when perception_stub.py is running; the agent will train but on meaningless vision features
replace perception_stub with a real node publishing 13440 floats on /<ns>/vision/keypoint_heatmap

obs_shape mismatch in config.yaml
state mode: obs_shape must equal 2 + heatmap_size; heatmap_size = 3*56*80 = 13440; total = 13442
if perception node outputs different heatmap shape, update obs_shape and vision_feature_size at gazebo.py:57

wandb not logging
check: enable_wandb: true in config.yaml; WANDB_API_KEY env var set; wandb_entity/wandb_project correct
disable cleanly: enable_wandb: false in config.yaml (logger falls back to csv only)
