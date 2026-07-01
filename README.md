<h1 align="center">
  LeCropFollow
</h1>

<h3 align="center">
  Latent Space Planning for Navigation in Unstructured Crop Fields
</h3>

<p align="center">
  <strong>Felipe Tommaselli</strong><sup>1</sup> &middot;
  <strong>Francisco Affonso</strong><sup>2</sup> &middot;
  <strong>Arthur Pompeu</strong><sup>1</sup> &middot;
  <strong>Gianluca Capezzuto</strong><sup>1</sup><br>
  <strong>Arun Narenthiran Sivakumar</strong><sup>2</sup> &middot;
  <strong>Girish Chowdhary</strong><sup>2</sup> &middot;
  <strong>Marcelo Becker</strong><sup>1</sup>
</p>

<p align="center">
  <sup>1</sup> University of Sao Paulo &nbsp;&nbsp;
  <sup>2</sup> University of Illinois Urbana-Champaign
</p>

<p align="center">
  <em>IEEE Robotics and Automation Letters, 2026</em>
</p>

<p align="center">
  <a href="https://arxiv.org/pdf/2606.31941">
    <img src="https://img.shields.io/badge/Paper-PDF-b31b1b?style=flat-square&logo=arxiv&logoColor=white" alt="Official Paper (soon)">
  </a>&nbsp;
  <a href="https://arxiv.org/abs/2606.31941">
    <img src="https://img.shields.io/badge/arXiv-2606.31941-b31b1b?style=flat-square&logo=arxiv&logoColor=white" alt="arXiv">
  </a>&nbsp;
  <a href="https://felipe-tommaselli.github.io/lecropfollow/">
    <img src="https://img.shields.io/badge/Project-Page-4285F4?style=flat-square&logo=google-chrome&logoColor=white" alt="Project Page">
  </a>&nbsp;
  <a href="https://youtu.be/hV1fDjQsgOs">
    <img src="https://img.shields.io/badge/Video-YouTube-FF0000?style=flat-square&logo=youtube&logoColor=white" alt="Video">
  </a>&nbsp;
  <a href="https://huggingface.co/datasets/arthurpompeu/lecrop-data">
    <img src="https://img.shields.io/badge/Data-HuggingFace-FFD21E?style=flat-square&logo=huggingface&logoColor=white" alt="Data">
  </a>&nbsp;
  <a href="https://api.wandb.ai/links/lecropfollow/mwd63kw7">
    <img src="https://img.shields.io/badge/Models-W%26B-FFBE00?style=flat-square&logo=weightsandbiases&logoColor=white" alt="Models">
  </a>
</p>

---

<p align="center"><img src="figures/cover.png" width="90%" alt="LeCropFollow overview"></p>

## Quickstart

LeCropFollow learns to navigate between crop rows by planning inside a latent world model over uncompressed semantic heatmaps. The approach requires no GNSS, performs no geometric estimation, and transfers from simulation to real hardware without fine-tuning (zero-shot sim-to-real).

This repository contains the full simulation training stack: a ROS Noetic + Gazebo environment, a TD-MPC2 reinforcement learning agent, and a plug-and-play interface for adapting to different robot platforms.

## Setup

**Prerequisites:** Docker and an NVIDIA GPU with drivers installed.

```bash
# 1. Verify NVIDIA container runtime
nvidia-smi
dpkg -l | grep nvidia-container-runtime   # install if missing

# 2. Build the image
bash docker/scripts/build.sh

# 3. Start the container
bash docker/scripts/run.sh
```

Inside the container, build the ROS workspace:

```bash
cd /ros_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

## Training

Set your [Weights & Biases](https://wandb.ai) API key and launch training. Gazebo starts automatically; metrics are logged to W&B.

```bash
export WANDB_API_KEY="your_key_here"

cd /ros_ws/lecropfollow/tdmpc2
python3 train.py
```

To suppress Gazebo and TIFF warnings from the console output:

```bash
python3 train.py 2>&1 | grep -v -E "XML Attribute|TIFFFetch|TIFFField"
```

**Resuming from a checkpoint.** Create `tdmpc2/checkpoints/`, place a `.pt` file inside, and set the `checkpoint` field in `config.yaml`.

## Adapting to a Different Robot

A complete topic contract between ROS and the RL agent is documented in [`SKILLS/integration.md`](SKILLS/integration.md). The minimal steps are:

1. **Set the namespace.** Point `robot_ns` in `tdmpc2/config.yaml` to your robot's topic prefix.

2. **Swap the URDF.** Replace `src/robot_description/urdf/robot.urdf.xacro` with your robot model.

3. **Integrate perception.** Replace `src/terra_utils/scripts/perception_stub.py` with a node that publishes real keypoint heatmap features on `/<robot_ns>/vision/keypoint_heatmap`.

## Citation

If you use this work in your research, please cite:

```bibtex
@article{tommaselli2026lecropfollow,
  title         = {LeCropFollow: Latent Space Planning for Navigation in Unstructured Crop Fields},
  author        = {Tommaselli, Felipe and Affonso, Francisco and Pompeu, Arthur and Capezzuto, Gianluca and Sivakumar, Arun Narenthiran and Chowdhary, Girish and Becker, Marcelo},
  journal       = {IEEE Robotics and Automation Letters},
  year          = {2026},
  note          = {Accepted for publication},
  eprint        = {2606.31941},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO}
}
```

This project builds on TD-MPC2. If you use the agent architecture, please also cite:

```bibtex
@inproceedings{hansen2024tdmpc2,
  title     = {TD-MPC2: Scalable, Robust World Models
               for Continuous Control},
  author    = {Nicklas Hansen and Hao Su and Xiaolong Wang},
  booktitle = {ICLR},
  year      = {2024}
}
```

## License

Code is released under the [MIT License](LICENSE). The paper is published under CC BY. See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.

## For AI Agents

If you are an AI coding agent working on this repository, start by reading the [`SKILLS/`](SKILLS/) folder. It contains codebase orientation, architecture diagrams, topic contracts, and debugging guides written specifically for you.
