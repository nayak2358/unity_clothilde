# Unity-Clothilde

Interactive Unity interface for the [Clothilde cloth simulator](https://github.com/fcoltraro/clothilde-sim), developed for cloth manipulation, virtual grasping, VR interaction, and trajectory data collection.

The project uses **Unity as the interaction and visualization layer**, while the cloth dynamics are computed by **Clothilde in Python**. Unity and Python communicate directly through Python.NET and shared memory buffers, allowing the simulated cloth state to be updated interactively.

## Features

- Real-time visualization of Clothilde simulations in Unity
- Direct manipulation of cloth vertices
- Virtual grippers for cloth grasping
- Desktop interaction through the `KeyboardScene`
- Bimanual manipulation using SteamVR in the `VRScene`
- Recording of cloth configurations and gripper trajectories
- Support for cloth-table interaction and configurable physical parameters

## Scenes

### KeyboardScene

The desktop scene can be used without a VR system. A virtual gripper can be translated and rotated using keyboard controls.

When the gripper closes, cloth nodes are selected from a finite grasping region. The selected nodes are subsequently constrained by Clothilde while the gripper moves.

This scene is particularly useful for debugging, testing grasp configurations, and generating reproducible manipulation trajectories.

### VRScene

The VR scene uses SteamVR controllers to provide two independently tracked six-degree-of-freedom grippers.

Each controller can grasp a region of the cloth and manipulate the corresponding vertices through positional constraints sent to Clothilde. This allows natural bimanual cloth manipulation and interactive collection of demonstrations.

Individual cloth vertices can also be manipulated using the VR pointer interface.

## Installation

### 1. Clone Clothilde

```bash
git clone https://github.com/fcoltraro/clothilde-sim.git
```

Create the Python environment:

```bash
conda create -n clothilde_env -c conda-forge \
    python=3.11 suitesparse scikit-sparse scipy numpy pykdtree

conda activate clothilde_env

pip install polyscope line_profiler
```

### 2. Clone this repository

```bash
git clone https://github.com/nayak2358/unity_clothilde.git
```

Open the project using **Unity Hub**.

### 3. Configure the Python connection

Update the local Python/Clothilde paths used by the Unity--Python connection to match your installation.

### 4. VR support

To use `VRScene`, install **SteamVR** and configure the SteamVR Unity plugin.

The `KeyboardScene` can be used without VR hardware.

## Data recording

Manipulation demonstrations can be exported from Unity. Recordings contain the cloth state together with the relevant gripper poses and grasp states, allowing the demonstrations to be processed independently of the Unity environment.

## Related project

Cloth simulation:

[Clothilde-Sim](https://github.com/fcoltraro/clothilde-sim)
