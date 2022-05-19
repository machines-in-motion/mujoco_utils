Bullet Utils
------

The code provided in this repo simplifies the interaction between pinocchio and mujoco.

### Installation

#### Standard dependencies

You need [dm_control](https://github.com/deepmind/dm_control and
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) to install this
package.

To run the demo, you need to install
[robot_properties_solo](https://github.com/open-dynamic-robot-initiative/robot_properties_solo)
as well.

#### Download the package

To download this package, you can

1. use `git clone`  
    ```
    mkdir -p ~/devel/workspace/src
    cd ~/devel/workspace/src
    git clone git@github.com:machines-in-motion/mujoco_utils.git
    ```

2. use [treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep) with the [treep_machines_in_motion](https://github.com/machines-in-motion/treep_machines_in_motion) configuration.  
    ```
    mkdir -p ~/devel
    pip3 install treep
    cd ~/devel
    git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
    treep --clone mujoco_utils
    ```

#### Build the package

You can install this package with 
- pure pip  
    ```
    cd mkdir -p ~/devel/workspace/src/mujoco_utils
    pip3 install .
    ```

- or [colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)  
    ```
    cd mkdir -p ~/devel/workspace
    colcon build
    ```

### Usage

#### Examples

To run the example
```
cd mkdir -p ~/devel/workspace/src/bullet_utils/demos
python3 solo12_impedane_standing.py
```

#### API documentation

The API documentation is automatically generated using colcon.
For the build to work one need to add the
[mpi_cmake_modules](https://github.com/machines-in-motion/mpi_cmake_modules)
to their workspace using git or treep.

```
cd ~/devel/workspace/src
git clone git@github.com:machines-in-motion/mpi_cmake_modules.git
```
or
```
treep --clone mpi_cmake_modules
```

TODO: *Where to find the last built doc on the internet.*

### License and Copyrights

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.


