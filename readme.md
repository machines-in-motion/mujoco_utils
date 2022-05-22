Mujoco Utils
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
cd mkdir -p ~/devel/workspace/src/mujoco_utils/examples
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

### Getting started doc

#### Converting URDF to Mujoco XML format

Assume you have already your robot specified in a URDF file, the conversion to Mujoco's XML format can be done automatically.

1. Open the Mujoco app
2. Drop the URDF file onto the app visualizer. Note that Mujoco ignores the path to the stl meshes and you might have to copy the stl meshes to the in the same folder as the URDF file
3. If things worked, you should now see the robot visualized in Mujoco
4. Click the "Save xml" button on the top left. This will export the URDF to xml
5. Grab the xml file. The file is stored next to the mujoco executeable. See the [hints here](https://github.com/deepmind/mujoco/issues/110) where to find it on macOS

At this point, you have the robot converted to xml. There are a few additional steps that you need to perform manually now to improve the modeling. You might want to take a look at the example xml of Solo12 in `examples/` to get an idea what is discussed in the following.

* The xml export only exports the robot. You might want to add a light for rendering and also a plane ground. You can do this by adding a `<light` and `<geom type="plane" ..` entry in the `<worldbody>`

```xml
    <worldbody>
        <light name="top" pos="0 0 1.5"/>
        <geom name="floor" type="plane" size="5 5 .1" condim="6" friction="0.6 0.005 0.0001" solref="0.015 1" solimp="0.99 0.99 0.001"/>
        <body name="solo12" pos="0 0 .5" childclass="solo12">
```

* For the initial import you needed the STL meshes next to the URDF file. You can now move the STL files into a subfolder. Do this and update the path in the `<asset>` `<mesh ...` entries:

```xml
<asset>
    <mesh name="solo_12_base" file="solo12_stl/solo_12_base.stl"/>
```

Here, the stl file was moved into the folder `solo12_stl` relative to the xml file

* You might want to change the default timestep:

```xml
    <option timestep="0.001"/>
```

* You might want to adjust the contact properties of the endeffectors. For this, specify the contact properties like in the following in the main `<mujoco> section:

```xml
    <default>
        <default class="contact">
            <geom condim="6" friction="0.6 0.005 0.0001" solref="0.015 1" solimp="0.99 0.99 0.001" priority="1"/>
        </default>
    </default>
```

For each `<geom>` object, that makes contact and should use the above contact properties, you have to add a `class="contact"` entry on the `<geom ...` tag. For instance:

```xml
<geom pos="0 -0.008 -0.16" type="mesh" rgba="0.8 0.8 0.8 1" mesh="solo_foot" class="contact"/>
```

Note the `contact` class entry at the end.

* In Mujoco XMLL, defining a joint doesn't mean you can actuate it directly yet. For this, you need to specify the `<actuator>` entry like this:

```xml
    <actuator>
        <motor name="FL_HAA"       gear="1"  joint="FL_HAA"/>
```

Note that Mujoco allows more complex joints than the one we show here.

* The imported model might have internal contacts between geometries, which is not desired. For instance on Solo12, Mujoco detected a contact between the main base and hte `FL_UPPER_LEG`. You can tell Mujoco to ignore such contacts by adding an entry for contact-exclusion

```xml
    <contact>
        <!-- Prevent contact between the base and other body parts. -->
        <exclude body1='solo12' body2='FL_UPPER_LEG'/>
```

How to find such internal contacts? Let your robot make contact with all endeffector with the ground. Then, look at the output of `physics.data.contact`. Then, look at the entries for `geom1` and `geom2`. In most cases, the geometry with index 0 is the floor. For example, here is the output with only the floor contact being detected:

```
  geom1: array([0, 0, 0, 0], dtype=int32)
  geom2: array([ 5,  9, 13, 17], dtype=int32)
```

While this is the output when removing the contact-exclusion in world_solo12.xml:

```
  geom1: array([0, 0, 0, 0, 1, 1, 1, 1], dtype=int32)
  geom2: array([ 5,  9, 13, 17,  3,  7, 11, 15], dtype=int32)
```

Note the detected contacts between the first geometry (the base) and the other geometires (1 <-> 3, 1 <-> 7, 1 <-> 11, 1 <-> 15).

#### Using the wrapper

To make working with Pinocchio from Mujoco easier, we provide a wrapper, which translates between the two world. For instance, in Mujoco, the quaternions are encoded differently `w x y z`, while Pinocchio takes them as `x y z w` coordinates.

You can create a robot wrapper like the following:

```python
# Load the model of Solo12 and create a mujoco simulation.
physics = mujoco.Physics.from_xml_path('world_solo12.xml')

# Load Solo12 as a pinocchio robot.
pin_robot = Solo12Config.buildRobotWrapper()

# Create wrapper to make interaction between Mujoco <-> Pinocchio easy.
jnt_names = [
    'base',
    'FL_HAA',
    'FL_HFE',
    'FL_KFE',
    'FR_HAA',
    'FR_HFE',
    'FR_KFE',
    'HL_HAA',
    'HL_HFE',
    'HL_KFE',
    'HR_HAA',
    'HR_HFE',
    'HR_KFE'
]
robot = MujocoWrapper(physics, pin_robot, jnt_names, [5, 9, 13, 17])
```

The indices `[5, 9, 13, 17]` are the geom ids of the endeffector that make contact (see above on how to get this ids from the `physics.data.contact` entry). Using the `robot.get_end_effector_forces()`, you can then get the forces and torques acting on the endeffector ids.

#### Using the visualizer

The `dm_control` package comes with a visualizer. See more details [here](https://github.com/deepmind/dm_control/tree/3e1736d7e641ab751eb25bfa7622fea71c8da9c6/dm_control/viewer#interactive-environment-viewer). The visualizer is great. However, it is not straight forward to use the visualizer without defining a `dm_control` suite. It is also not possible to step the simulator / application in a sequential fashion / without using a callback function.

To overcome these issues, a `MujocoApplication` is provided in this package. This application builds on top of the viewer provided by `dm_control`. Having created a `MujocoWrapper` before, the following shows how to create an app and run a simple PD controller in a control loop:

```python
app = MujocoApplication(physics, frame_rate=24)

# 10 s simulation
N = 1000

robot.reset_state(Solo12Config.q0, Solo12Config.v0)

start = time.time()

for i in range(N):
# while True:
    q, dq = robot.get_state()
    jnt_des = np.array([0., 0.8, -1.6, 0., 0.8, -1.6, 0., -0.8, 1.6, 0., -0.8, 1.6])
    tau = 5. * (jnt_des - q[7:]) - 0.05 * dq[6:]
    robot.send_joint_command(tau)

    # Uses a sleep factor of 2x:
    # Means the robot runs 2x as fast as realtime assuming the rendering time
    # and running the controller allows it in the time budget.
    app.step(sleep=2)

# Force a redraw to get the latest state of the robot in sync with the view.
app.redraw()

print(time.time() - start)
app.close()
```

A few remarks:

* Like PyBullet, the visualizer allows for perturbations with the mouse. For this, double click on the body you want to perturbe and hold down the ctrl key, press the mouse and move. A red pertubation arrow should appear and the robot move.

* The visualizer is rendered by calling the `app.step` accordingly to the specify frame_rate. To make the visualization be aligned with the latest state of the robot, it's important to call the `app.redraw` at the end of the control loop

* Doing the rendering can take up quite a bit of time. If you want to get maximum speed of the simulation, you can do the following: Reduce the frame rate of the application or/and reduce the window width and height. In practise, a frame_rate of 12 might feel good enough for everyday research, while 24 should be used for publication videos.

* If you want to render async, the recommendation is to use Meshcat as visualizer and step the physics/robot yourself (get the state from the robot, apply it to meshcat etc)

* The `sleep` option on the `app.step` method has a few optoins.
    * You can either not sleep at all by specifing `app.step(sleep=False)`. This will step the simulator as fast as possible while doing the rendering at the specified app's frame rate.
    * If you specify `app.step(sleep=True)`, the app will sleep after each step for `dt` time as provided by `physics.timestep()`
    * If you specify a float like `app.step(sleep=1.)`, the app will try to render and step x time faster than specified. For instance, when sleep=1., then the app will try to run at realtime speed (so 10 s simulation should take around 10 s in the app). When you specify `sleep=2`, the simulation will step twice as fast as realtime (10 s should run in 5 s in the app). Note that this is limited by the speed of executing the controller and the rendering speed. You can also use this setting to slow down, by e.g. specifing `sleep=0.1`, which makes 1 s in simulation beeing rendered over 10 seconds.

### License and Copyrights

License BSD-3-Clause
Copyright (c) 2022, New York University.
