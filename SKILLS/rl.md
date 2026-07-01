primary files: tdmpc2/train.py, tdmpc2/config.yaml, tdmpc2/envs/gazebo.py, tdmpc2/tdmpc2.py

train.py flow:
1. parse_cfg(cfg) — merges hydra config, resolves obs_shape/action_dim/episode_length
2. make_env(cfg) → GazeboEnv — spawns roscore+roslaunch inside __init__, waits 8s
3. TDMPC2(cfg) — instantiates world model + actor + critics
4. OnlineTrainer.train() loop:
   steps 0..seed_steps: random actions (env.rand_act()), experience stored
   step seed_steps: pretraining on seed buffer (seed_steps update passes)
   steps seed_steps+1..: agent.act(obs) → env.step(action) → agent.update(buffer)
   every eval_freq steps: OnlineTrainer.eval() runs eval_episodes episodes
5. trainer.close() — kills ROS/Gazebo via common/shutdown.py

config.yaml key params (file: tdmpc2/config.yaml):
robot_ns       topic prefix, must match terra_gazebo.launch arg (default: robot)
obs            "state" (13442-dim vector) or "rgb" (3×image_size×image_size)
obs_shape      13442 for state mode = 2 action floats + 3×56×80 heatmap
action_dim     2 (linear_x, angular_z)
launchfile     relative path from tdmpc2/envs/ to the .launch file
checkpoint     optional .pt path; if set, agent.load() is called before training
steps          total env steps (default 300000)
seed_steps     random exploration steps before first update (default 5500)
batch_size     512
eval_freq      30000
max_distance   episode ends when robot.x > this (metres, default 120)
max_ep_steps   episode length cap (default 500)

GazeboEnv (tdmpc2/envs/gazebo.py):
__init__:L45   sets robot_ns, obs_type, action_space, observation_space; spawns ROS; creates all pub/sub
step():L154    normalize_actions → EMA filter → publish Twist → unpause → sleep(STEP_DELAY=0.085) → pause → build obs → reward
reset():L220   reset_world service → unpause → sleep(RESET_DELAY=2.0) → pause → return obs
observe_collision():L259  checks dis_error > 0.62m, vel_x stall, pitch/roll > 0.01rad
get_reward():L288  logistic softplus: dense_rw=exp(-(v-1)²/0.1) / (1+exp(0.6*discounts))

observation assembly (state mode):
obs[0:2]   filtered_action (EMA-smoothed commanded vel, linear+angular)
obs[2:]    heatmap_features from /robot_ns/vision/keypoint_heatmap (13440 floats, 3×56×80)
note: reset() uses [vel_x, yaw] for obs[0:2] instead of filtered_action — known inconsistency

action denormalisation (in normalize_actions()):
action[0] linear:  [-1,1] → [0.1, 1.0] m/s
action[1] angular: [-1,1] → [-0.9, 0.9] rad/s
then EMA: filtered = 0.3*action + 0.7*filtered_prev

agent files:
tdmpc2/tdmpc2.py         act(obs, t0, eval_mode) → action tensor; update(buffer) → metrics dict
tdmpc2/common/buffer.py  Buffer.add(td) stores TensorDict episodes; Buffer.sample(batch_size)
tdmpc2/common/logger.py  Logger.log(metrics, mode) → wandb + csv; Logger.video for episode recording
tdmpc2/common/parser.py  parse_cfg: fills obs_shapes/action_dims/episode_lengths from env introspection
