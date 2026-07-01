import dataclasses
import os
import datetime
import re

import numpy as np
import pandas as pd
from termcolor import colored
import torch

import colorful as cf

CONSOLE_FORMAT = [
	("episode", "E", "int"),
	("step", "I", "int"),
	("episode_reward", "R", "float"),
	("episode_success", "S", "float"),
	("total_time", "T", "time"),
]

CAT_TO_COLOR = {
	"train": "blue",
	"eval": "green",
}

def make_dir(dir_path):
	"""Create directory if it does not already exist."""
	try:
		os.makedirs(dir_path)
	except OSError:
		pass
	return dir_path

def print_run(cfg):
	"""
	Pretty-printing of current run information.
	Logger calls this method at initialization.
	"""
	prefix, color, attrs = "  ", "green", ["bold"]

	def _limstr(s, maxlen=36):
		return str(s[:maxlen]) + "..." if len(str(s)) > maxlen else s

	def _pprint(k, v):
		print(
			prefix + colored(f'{k.capitalize()+":":<15}', color, attrs=attrs), _limstr(v)
		)

	observations  = ", ".join([str(v) for v in cfg.obs_shape.values()])
	kvs = [
		("task", cfg.task_title),
		("steps", f"{int(cfg.steps):,}"),
		("observations", observations),
		("actions", cfg.action_dim),
		("experiment", cfg.exp_name),
	]
	w = np.max([len(_limstr(str(kv[1]))) for kv in kvs]) + 25
	div = "-" * w
	print(div)
	for k, v in kvs:
		_pprint(k, v)
	print(div)

def cfg_to_group(cfg, return_list=False):
	"""
	Return a wandb-safe group name for logging.
	Optionally returns group name as list.
	"""
	lst = [cfg.task, re.sub("[^0-9a-zA-Z]+", "-", cfg.exp_name)]
	return lst if return_list else "-".join(lst)

class DualHeatmapTracker:
	"""
	Tracks both position visitation and value estimates during evaluation.
	Creates two heatmaps logged to WandB.
	"""
		
	def __init__(self, cfg, wandb, agent, x_bins=30, y_bins=3):
		self.enabled = wandb is not None
		self._wandb = wandb
		self.agent = agent
		self.cfg = cfg
		self.x_bins = x_bins  # 30 cells × 2m = 60m
		self.y_bins = y_bins  # 3 lanes: left/center/right
		self.max_x = cfg.max_distance  # 60m
		self.y_threshold = 0.62  # lateral threshold
		
		# Storage for trajectories
		self.trajectories = []  # List of (x, y) lists for each episode
		self.current_trajectory = []
		
		# Heatmaps
		self.position_heatmap = np.zeros((y_bins, x_bins))
		self.value_heatmap = np.zeros((y_bins, x_bins))
		self.value_counts = np.zeros((y_bins, x_bins))  # For averaging
		
	def reset(self):
		"""Reset for new evaluation run (all episodes)"""
		self.trajectories = []
		self.current_trajectory = []
		self.position_heatmap = np.zeros((self.y_bins, self.x_bins))
		self.value_heatmap = np.zeros((self.y_bins, self.x_bins))
		self.value_counts = np.zeros((self.y_bins, self.x_bins))
		
	def start_episode(self):
		"""Start tracking a new episode"""
		self.current_trajectory = []
		
	def record_step(self, obs, x_pos, y_error):
		"""
		Record a single step during evaluation.
		
		Args:
			obs: Current observation tensor
			x_pos: Current x position (0-60m)
			y_error: Lateral deviation error
		"""
		if not self.enabled:
			return
		
		# Discretize position
		x_idx = min(int(x_pos / 2.0), self.x_bins - 1)
		if x_idx < 0:
			x_idx = 0
			
		if y_error < -self.y_threshold / 2:
			y_idx = 0  # left
		elif y_error > self.y_threshold / 2:
			y_idx = 2  # right
		else:
			y_idx = 1  # center
		
		# Update position heatmap
		self.position_heatmap[y_idx, x_idx] += 1
		
		# Store trajectory point
		self.current_trajectory.append((x_pos, y_error))
		
		# Estimate value at this position
		value = self._estimate_value_at_state(obs)
		if value is not None:
			self.value_heatmap[y_idx, x_idx] += value
			self.value_counts[y_idx, x_idx] += 1
		
	def end_episode(self):
		"""Finish tracking current episode"""
		if self.current_trajectory:
			self.trajectories.append(self.current_trajectory.copy())
			self.current_trajectory = []
		
	@torch.no_grad()
	def _estimate_value_at_state(self, obs):
		"""
		Estimate value function at current state using agent's Q-function.
		
		Returns approximate V(s) = Q(s, π(s))
		"""
		try:
			# Encode observation to latent state
			obs_tensor = obs.to(self.agent.device).unsqueeze(0)
			z = self.agent.model.encode(obs_tensor, task=None)
			
			# Get policy action
			action, _ = self.agent.model.pi(z, task=None)
			
			# Get Q-value for this state-action pair
			q_value = self.agent.model.Q(z, action, task=None, return_type='avg')
			
			return q_value.item()
		except Exception as e:
			# Silently fail if value estimation fails
			return None
		
	def save(self, step):
		"""Generate and log both heatmaps to WandB"""
		if not self.enabled:
			return
		
		import matplotlib.pyplot as plt
		
		# Create figure with 2 subplots
		fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
		
		# === VISUALIZATION 1: Position + Trajectory ===
		im1 = ax1.imshow(
			self.position_heatmap, 
			cmap='YlOrRd', 
			aspect='auto',
			extent=[0, self.max_x, -1, 1],
			origin='lower',
			alpha=0.7
		)
		
		# Overlay trajectories
		if self.trajectories:
			colors = plt.cm.viridis(np.linspace(0, 1, len(self.trajectories)))
			for i, traj in enumerate(self.trajectories):
				if traj:
					xs, ys = zip(*traj)
					ax1.plot(xs, ys, color=colors[i], alpha=0.6, linewidth=1.5)
		
		ax1.set_xlabel('X Position (m)', fontsize=12)
		ax1.set_ylabel('Lateral Position', fontsize=12)
		ax1.set_yticks([-0.67, 0, 0.67])
		ax1.set_yticklabels(['Left', 'Center', 'Right'])
		ax1.set_title('Position Heatmap with Trajectories', fontsize=14, fontweight='bold')
		ax1.grid(True, alpha=0.3)
		plt.colorbar(im1, ax=ax1, label='Visit Count')
		
		# === VISUALIZATION 2: Value Function ===
		# Average the values
		avg_value_heatmap = np.divide(
			self.value_heatmap, 
			self.value_counts,
			out=np.zeros_like(self.value_heatmap),
			where=self.value_counts > 0
		)
		
		im2 = ax2.imshow(
			avg_value_heatmap,
			cmap='RdYlGn',  # Red (low) -> Yellow -> Green (high)
			aspect='auto',
			extent=[0, self.max_x, -1, 1],
			origin='lower'
		)
		
		ax2.set_xlabel('X Position (m)', fontsize=12)
		ax2.set_ylabel('Lateral Position', fontsize=12)
		ax2.set_yticks([-0.67, 0, 0.67])
		ax2.set_yticklabels(['Left', 'Center', 'Right'])
		ax2.set_title('Learned Value Function Estimate (Q-values)', fontsize=14, fontweight='bold')
		ax2.grid(True, alpha=0.3)
		plt.colorbar(im2, ax=ax2, label='Avg Q-Value')
		
		plt.tight_layout()
		
		# Log to WandB
		self._wandb.log({
			"eval/position_trajectory_heatmap": self._wandb.Image(fig),
		}, step=step)
		
		plt.close(fig)

class VideoRecorder:
	"""Utility class for logging evaluation videos."""

	def __init__(self, cfg, wandb, fps=15):
		self.cfg = cfg
		self._save_dir = make_dir(cfg.work_dir / 'eval_video')
		self._wandb = wandb
		self.fps = fps
		self.frames = []
		self.enabled = False

	def init(self, env, enabled=True):
		self.enabled = self._save_dir and self._wandb and enabled
		self.frames = []

	def record(self, env):
		if self.enabled:
			frame = np.array(env.last_heatmap_image)
			self.frames.append(frame)

	def save(self, step, ep_travel_dists, ep_collision, key='videos/eval_video'):
		if self.enabled and len(self.frames) > 0:
			frames = np.stack(self.frames)
			frames = frames.astype(np.uint8)
			caption = list(zip(ep_collision, ep_travel_dists))

			self._wandb.log(
				{key: self._wandb.Video(frames.transpose(0, 3, 1, 2), fps=self.fps, format='mp4', caption=f"Collisions and distances: {caption}")}, step=step)

class Logger:
	"""Primary logging object. Logs either locally or using wandb."""

	def __init__(self, cfg):
		self._log_dir = make_dir(cfg.work_dir)
		self._model_dir = make_dir(self._log_dir / "models")
		self._save_csv = cfg.save_csv
		self._save_agent = cfg.save_agent
		self._group = cfg_to_group(cfg)
		self._seed = cfg.seed
		self._eval = []

		#TODO: change this to multiple training formats
		self.identifier = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

		print_run(cfg)
		self.project = cfg.get("wandb_project", "none")
		self.entity = cfg.get("wandb_entity", "none")
		if not cfg.enable_wandb or self.project == "none" or self.entity == "none":
			print(colored("Wandb disabled.", "blue", attrs=["bold"]))
			cfg.save_agent = False
			cfg.save_video = False
			self._wandb = None
			self._video = None
			return
		os.environ["WANDB_SILENT"] = "true" if cfg.wandb_silent else "false"
		import wandb

		wandb.init(
			project=self.project,
			entity=self.entity.encode('ascii', 'replace').decode(),
			name= str(self.identifier), #str(cfg.seed),
			group=self._group,
			tags=cfg_to_group(cfg, return_list=True) + [f"seed:{cfg.seed}"],
			dir=self._log_dir,
			config=dataclasses.asdict(cfg),
		)
		print(colored("Logs will be synced with wandb.", "blue", attrs=["bold"]))
		self._wandb = wandb
		self._video = (
			VideoRecorder(cfg, self._wandb)
			if self._wandb and cfg.save_video
			else None
		)
		self._heatmap_tracker = None  # Will be initialized with agent reference
		self.cfg = cfg  # Store config for heatmap tracker

	@property
	def video(self):
		return self._video

	@property
	def heatmap_tracker(self):
		return self._heatmap_tracker
		
	def init_heatmap_tracker(self, agent):
		"""Initialize heatmap tracker with agent reference"""
		if self._wandb:
			self._heatmap_tracker = DualHeatmapTracker(
				cfg=self.cfg,
				wandb=self._wandb,
				agent=agent
			)
			print(colored("Heatmap tracker initialized.", "blue", attrs=["bold"]))

	def save_agent(self, agent=None):
		if self._save_agent and agent:
			# Save model
			model_fp = self._model_dir / f'{str(self.identifier) + "-final"}.pt'
			print(cf.bold_red(f'Saving model to {model_fp}'))
			agent.save(model_fp)

			if self._wandb:
				# Log model artifact
				model_artifact = self._wandb.Artifact(
					self._group + '-' + str(self.identifier) + '-model',
					type='model'
				)
				model_artifact.add_file(model_fp)
				self._wandb.log_artifact(model_artifact)

	def finish(self, agent=None):
		try:
			self.save_agent(agent)
		except Exception as e:
			print(colored(f"Failed to save model: {e}", "red"))
		if self._wandb:
			self._wandb.finish()

	def _format(self, key, value, ty):
		if ty == "int":
			return f'{colored(key+":", "blue")} {int(value):,}'
		elif ty == "float":
			return f'{colored(key+":", "blue")} {value:.01f}'
		elif ty == "time":
			value = str(datetime.timedelta(seconds=int(value)))
			return f'{colored(key+":", "blue")} {value}'
		else:
			raise ValueError(f"invalid log format type: {ty}")

	def _print(self, d, category):
		category = colored(category, CAT_TO_COLOR[category])
		pieces = [f" {category:<14}"]
		for k, disp_k, ty in CONSOLE_FORMAT:
			if k in d:
				pieces.append(f"{self._format(disp_k, d[k], ty):<22}")
		print("   ".join(pieces))

	def log(self, d, category="train"):
		assert category in CAT_TO_COLOR.keys(), f"invalid category: {category}"
		if self._wandb:
			xkey = "step"
			_d = dict()
			for k, v in d.items():
				_d[category + "/" + k] = v
			self._wandb.log(_d, step=d[xkey])
		if category == "eval" and self._save_csv:
			keys = ["step", "episode_reward"]
			self._eval.append(np.array([d[keys[0]], d[keys[1]]]))
			pd.DataFrame(np.array(self._eval)).to_csv(
				self._log_dir / "eval.csv", header=keys, index=None
			)
		self._print(d, category)
