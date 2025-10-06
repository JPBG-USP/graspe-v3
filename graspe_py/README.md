# Grasp-e Py

**Grasp-e Py** is the official Python package for the **Grasp-e robotic manipulator**, designed for simulation, control, and analysis. It integrates modeling, motor simulation, and kinematic tools for research and development in robotic manipulation.

## Features

- Modular architecture for simulation and control
- Easy configuration using YAML environment files
- Support for DC motor modeling and kinematic analysis
- Compatible with [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/JPBG-USP/graspe-v3.git
cd graspe-v3
```

### 2. Create a conda Environment

The repository provides an environment file named environment.yaml with all required dependencies.

```bash
conda env create -f environment.yaml
conda activate graspe
```

### 3. Install the package in editable mode
This allows live updates when editing the source code.

```bash
pip install -e grape_py/
```

## Running the simulation
To run the manipulator simulation using the discret PID controller, you can run:

```bash
python graspe_py/src/graspe_py/run_simulation.py
```

You can choose the simulation time by passin `--time`

```bash
python graspe_py/src/graspe_py/run_simulation.py --time 5.0
``` 