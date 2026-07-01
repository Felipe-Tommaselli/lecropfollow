# Contributing

The paper is published under CC BY (no restrictions). The code in this repo is MIT licensed. Both can coexist — CC BY applies to the academic work, MIT applies to the software.

## Ways to contribute

Robot integrations: swap `src/robot_description/urdf/robot.urdf.xacro` and open a PR with a new robot. Document what changed in `SKILLS/ros.md`.

Perception models: replace `src/terra_utils/scripts/perception_stub.py` with a real keypoint or feature extraction node. The interface contract is in `SKILLS/ros.md` — your node must publish a `std_msgs/Float32MultiArray` of shape (13440,) on `/<robot_ns>/vision/keypoint_heatmap`.

New Gazebo worlds: add `.world` files and models under `src/terra_worlds/`. Point to them via the `world` arg in `terra_gazebo.launch`.

Bug reports: open an issue with the full error, the output of `rostopic list`, and your `config.yaml`. Check `SKILLS/debug.md` first.

## Pull requests

1. Fork and branch from `main`.
2. Keep changes focused — one concern per PR.
3. If you touch `gazebo.py` or `config.yaml`, verify `python3 train.py` reaches the seed phase without error.
4. If you add a new robot or perception node, add an entry to `SKILLS/debug.md` for known failure modes.

## Code style

No formatter is enforced. Match the style of the file you are editing. No unnecessary comments — the `SKILLS/` folder is the right place for explanations.

## License

By contributing, you agree your code will be licensed under MIT (see `LICENSE`).
