[![Tests](https://github.com/ADVRHumanoids/xbot2_py_bridge/actions/workflows/test.yml/badge.svg)](https://github.com/ADVRHumanoids/xbot2_py_bridge/actions/workflows/test.yml)
# xbot2_py_bridge

Python bridge for XBot2.

## Installation

```bash
pip install xbot2_py_bridge
```

For development:

```bash
pip install -e ".[dev]"
```

## Usage

```python
import xbot2_py_bridge
```

## Running tests

```bash
pip install -e ".[dev]"
python -m pytest
```

## Docker build test

The build workflow uses forest inside the XBot2 development images for Ubuntu
Noble (ROS 2 Jazzy) and Resolute (ROS 2 Lyrical). To run it locally:

```bash
cd docker/noble-ros2 # or docker/resolute-ros2
docker compose up -d --build
docker compose exec -T dev bash ./scripts/build.bash
docker compose down --volumes
```

Set `FOREST_JOBS` before starting the container to change build parallelism
(default: 1). The build uses the image's bundled XBot2 and builds the JSON
dependency with forest; it does not fetch or build XBot2 from source.
