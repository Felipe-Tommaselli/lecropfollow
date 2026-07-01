# System and environment variables
import os
import warnings
os.environ['LAZY_LEGACY_OP'] = '0'
os.environ['TORCHDYNAMO_INLINE_INBUILT_NN_MODULES'] = "1"
os.environ['TORCH_LOGS'] = "+recompiles"
warnings.filterwarnings('ignore')

# PyTorch imports and settings
import torch
torch.set_default_dtype(torch.float32)
torch.backends.cudnn.benchmark = True
torch.set_float32_matmul_precision('high')

# Core libraries
import numpy as np
from time import time

# Third party libraries
import hydra
import gymnasium as gym
from termcolor import colored
import colorful as cf
from tensordict.tensordict import TensorDict
gym.logger.set_level(40)

# Local imports
from common.parser import parse_cfg
from common.seed import set_seed
from common.buffer import Buffer
from common.logger import Logger
from common.shutdown import shutdown_ros_and_gazebo
from tdmpc2 import TDMPC2
from envs.gazebo import make_gazebo_env

####################################
# Online training for TD-MPC2 agent#
####################################
class OnlineTrainer:
	"""Trainer class for single-task online TD-MPC2 training."""

	def __init__(self, cfg, env, agent, buffer, logger):
		self.cfg = cfg
		self.env = env
		self.agent = agent
		self.buffer = buffer
		self.logger = logger
		self._step = 0
		self._ep_idx = 0
		
		self.logger.init_heatmap_tracker(agent)
		self._start_time = time()

	def common_metrics(self):
		"""Return a dictionary of current metrics."""
		return dict(
			step=self._step,
			episode=self._ep_idx,
			total_time=time() - self._start_time,
		)

	def eval(self):
		"""Evaluate a TD-MPC2 agent."""
		ep_rewards, ep_travel_dists, ep_collision= [], [], []
		self.env.in_eval = True

		# Initialize heatmap tracking   
		if self.logger.heatmap_tracker:
			self.logger.heatmap_tracker.reset()

		if self.cfg.save_video:
				self.logger.video.init(self.env, enabled=True)

		for i in range(self.cfg.eval_episodes):
			# Start new episode trajectory
			if self.logger.heatmap_tracker:
				self.logger.heatmap_tracker.start_episode()
			
			obs, done, ep_reward, t = self.env.reset(), False, 0, 0
			while not done:
				torch.compiler.cudagraph_mark_step_begin()
				action = self.agent.act(obs, t0=t==0, eval_mode=True)
				
				# Record position and value BEFORE step
				if self.logger.heatmap_tracker:
					self.logger.heatmap_tracker.record_step(
						obs=obs,
						x_pos=self.env.current_x,
						y_error=self.env.dis_error
					)
				
				obs, reward, done, info = self.env.step(action)
				ep_reward += reward
				t += 1

				if self.cfg.save_video:
					self.logger.video.record(self.env)

			# End episode trajectory
			if self.logger.heatmap_tracker:
				self.logger.heatmap_tracker.end_episode()
			
			ep_rewards.append(ep_reward)
			ep_travel_dists.append(round(info['travel_dist'], 2))
			ep_collision.append(info['collision_type'])
			if self.cfg.save_video:
				self.logger.video.save(self._step, ep_travel_dists, ep_collision)

		if self.logger.heatmap_tracker:
			self.logger.heatmap_tracker.save(self._step)
		self.env.in_eval = False

		return dict(
			episode_reward = np.nanmean(torch.stack([r.cpu() for r in ep_rewards]).numpy()),
			episode_success=np.nanmean(ep_travel_dists),
		)

	def to_td(self, obs, action=None, reward=None):
		"""Creates a TensorDict for a new episode."""
		device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
		if isinstance(obs, dict):
			obs = TensorDict(obs, batch_size=(), device=device)
		else:
			obs = obs.unsqueeze(0).to(device)
		if action is None:
			action = torch.full_like(self.env.rand_act(), float('nan')).to(device)
		if reward is None:
			reward = torch.tensor(float('nan')).to(device)
		td = TensorDict(
			obs=obs,
			action=action.unsqueeze(0).to(device),
			reward=reward.unsqueeze(0).to(device),
			batch_size=(1,))
		return td

	def train(self):
		"""Train a TD-MPC2 agent."""
		train_metrics, done, eval_next = {}, True, False

		while self._step <= self.cfg.steps:
			# Evaluate agent periodically
			if ((self._step % self.cfg.eval_freq == 0) and (self._step > 0)) or self.cfg.eval_on_every_reset:
				eval_next = True

			# Reset environment
			if done:
				if eval_next:
					eval_metrics = self.eval()
					eval_metrics.update(self.common_metrics())
					self.logger.log(eval_metrics, 'eval')
					eval_next = False

					if self.cfg.eval_on_every_reset:
						obs = self.env.reset()
						self._tds = [self.to_td(obs)]
						self._step += 1
						continue
						
				if self._step > 0:
					train_metrics.update(
						episode_reward=torch.tensor([td['reward'] for td in self._tds[1:]]).sum(),
						episode_success=round(info['travel_dist'], 2),
					)
					train_metrics.update(self.common_metrics())
					self.logger.log(train_metrics, 'train')
					self._ep_idx = self.buffer.add(torch.cat([td.to('cuda' if torch.cuda.is_available() else 'cpu') for td in self._tds], dim=0))

				obs = self.env.reset()
				self._tds = [self.to_td(obs)]

			# Collect experience
			if self._step > self.cfg.seed_steps:
				action = self.agent.act(obs, t0=len(self._tds)==1)
			else:
				action = self.env.rand_act()

			obs, reward, done, info = self.env.step(action)

			self._tds.append(self.to_td(obs, action, reward))

			# Update agent
			if self._step >= self.cfg.seed_steps:
				if self._step == self.cfg.seed_steps:
					num_updates = self.cfg.seed_steps
					print('Pretraining agent on seed data...')
				else:
					num_updates = 1

				for _ in range(num_updates):
					_train_metrics = self.agent.update(self.buffer)
					train_metrics.update(_train_metrics)

			self._step += 1

		self.logger.finish(self.agent)

	def close(self):
		"""Gracefully shut down the environment and ROS/Gazebo."""
		print(colored("[trainer] Shutting down...", "yellow"))
		
		# Close environment if it has a close method
		if hasattr(self.env, "close"):
			try:
				self.env.close()
			except Exception as e:
				print(colored(f"[trainer] env.close() failed: {e}", "red"))
		
		# Force-kill ROS/Gazebo processes
		shutdown_ros_and_gazebo()

##########################################
# Environment creation for TD-MPC2 agent #
##########################################
def make_env(cfg):
	"""
	Make an environment for TD-MPC2 experiments.
	Based on FOWM creational pattern.
	"""

	env = make_gazebo_env(cfg)

	if env is None:
		raise ValueError(f'Failed to make environment "{cfg.task}": please verify that dependencies are installed and that the task exists.')
	try: # Dict
		cfg.obs_shape = {k: v.shape for k, v in env.observation_space.spaces.items()}
	except: # Box
		cfg.obs_shape = {cfg.get('obs', 'state'): [cfg.get("obs_shape")]}
	cfg.action_dim = cfg.get("action_dim")
	cfg.episode_length = cfg.get("max_ep_steps") 
	cfg.seed_steps = max(1000, 5*cfg.episode_length)
	print(f'::::::::::: SEED STEPS: {cfg.seed_steps} :::::::::::::')
	return env

#########################
# Primmary train caller #
#########################
@hydra.main(config_name='config', config_path='.')
def train(cfg: dict):
	"""
	Script for training single-task TD-MPC2 agents.
	"""
	assert torch.cuda.is_available()
	assert cfg.steps > 0, 'Must train for at least 1 step.'
	cfg = parse_cfg(cfg)
	set_seed(cfg.seed)
	print(colored('Work dir:', 'yellow', attrs=['bold']), cfg.work_dir)

	env = make_env(cfg)
	agent = TDMPC2(cfg)
	if cfg.get('checkpoint', None):
		assert os.path.exists(cfg.checkpoint), f'Checkpoint {cfg.checkpoint} not found! Must be a valid filepath.'
		print(colored(f'Checkpoint: {cfg.checkpoint}', 'blue', attrs=['bold']))
		agent.load(cfg.checkpoint)
	else: 
		print(colored('No checkpoint provided.', 'red', attrs=['bold']))

	trainer = OnlineTrainer(
		cfg=cfg,
		env=env,
		agent=agent,
		buffer=Buffer(cfg),
		logger=Logger(cfg),
	)
	
	try:
		trainer.train()
		print('\nTraining completed successfully')
	finally:
		# Always clean up ROS/Gazebo, even if training fails
		trainer.close()

if __name__ == '__main__':
	train()
