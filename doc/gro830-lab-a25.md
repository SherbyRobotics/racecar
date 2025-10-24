# GRO830-A25 - Lab Issues & Tweaks

> _N.B._ This section documents a "Band-Aid" to fix the issues with GRO830's lab. The fix described ARE temporary, and a robust solution MUST be developped.

This section documents an issue occuring on the faculty's lab computers during A25. If you run into this issue on your personnal computer or on your Raspberry Pi 5, follow [this section](#sudo-privileges).

## Fixing the issue

### Sudo privileges

If you have `sudo` privileges, first try to update the package versions:

```bash
sudo apt update
sudo apt upgrade
```

If the issue persists, try updating the dependencies with `rosdep`:

```bash
cd ~/ros2_ws
sudo rosdep init  # If and only if you did NOT already run this command
rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
```

If the issue persists, follow [this section](#band-aid).

### Band-Aid

To fix the issue, you MUST create and use a Python virtual environment (`venv`). This enables local installation of Python packages with newer versions without requiring `sudo` privileges.

To create the Python `venv`, execute the following commands:

```bash
cd ~/ros2_ws
python3 -m venv .venv  # Create the virtual environment
source .venv/bin/activate  # Activate the virtual environment
```

> IMPORTANT: you MUST activate the virtual environment each time you open a new terminal.

Once the virtual environment is activated, install the dependencies:

```bash
pip3 install --requirement src/racecar/gro830-lab.txt
```

Verify the installation worked with `pip3 list | grep transforms3D`. The version SHOULD be `0.4.2`.

The ROS2 build procedure is now the following:

```bash
source .venv/bin/activate  # If and only if the venv is NOT yet activated
colcon build [--symlink-install]
source install/local_setup.bash  # Only on the first build
```

> _N.B._ The script WILL work if the `venv` is activated after the build

## Executing scripts

When trying to run the _brushfire_ script from GRO830's lab:

```bash
ros2 run racecar_behaviors labo_brushfire
```

You may run into an error similar to the following:

```bash
File "/usr/local/lib/python3.12/dist-packages/transforms3d/__init__.py", line 10, in <module>
   from . import quaternions
File "/usr/local/lib/python3.12/dist-packages/transforms3d/quaternions.py", line 26, in <module>
   _MAX_FLOAT = np.maximum_sctype(np.float64)
 File "/usr/local/lib/python3.12/site-packages/numpy/__init__.py", line 397, in __getattr__
   raise AttributeError(
AttributeError: `np.maximum_sctype` was removed in the NumPy 2.0 release. Use a specific dtype instead. You should avoid relying on any implicit mechanism and select the largest dtype of a kind explicitly in the code.
```

The cause of this issue is unknown, but it has to do with how Python finds its dependencies. The error is hard to replicate, because it seems to depend on a certain path to NumPy being higher in the list of paths in `sys.path`, meaning one computer could avoid the error entirely if the problematic path appears after the "known good" path in the list.

This issue only occurs when using `ros2 run` to execute the script, so the workaround is to use `python3`:

```bash
python3 path/to/labo_brushfire.py
```

Alternatively, if you want `ros2 run` to work, you can uncomment the lines of code at the top of the file to remove the problematic path from the list, BUT this "Band-Aid" is a very bad solution, and is NOT recommended. Note that this solution works exclusively for this script, and you need to paste the lines in every script (affected by the issue) executed using either `ros2 run` or `ros2 launch`.

## Tweaks to apply on the faculty's lab PCs

The first tweak is to remove the now deprecated environment variable Gazebo used to find its resources. For A25, the version of Gazebo used is "Harmonic", which uses the environment variable `GZ_SIM_RESOURCE_PATH`. However, the ROS image for the racecar has yet to update the environment variable, and still uses `IGN_GAZEBO_RESOURCE_PATH`. To apply the tweak, run the following command **on each new session on a lab PC**:

```bash
echo "unset IGN_GAZEBO_RESOURCE_PATH" >> ~/.bashrc
```

> NOTES:
>
> - A session is ended whenever the user is disconnected;
> - Remember to close the terminal you used to apply the tweak BEFORE using ROS;
> - Though it doesn't seem to cause the issues observed on the faculty's lab PCS, it is still **recommended** to apply this tweak on the Raspberry Pi (since the `.bashrc` file isn't erased when a session ends, you only need to apply it ONCE).

The second tweak is not required, but it saves on build time. Because the lab PCs are only used to run Gazebo simulations and do not use serial communication to send the low-level commands to the racecar, two packages can be skipped in the build. To skip these packages, add `--packages-skip serial pb2roscpp` at the end of your `colcon build` command. Here's an example of the full command:

```bash
colcon build --symlink-install --packages-skip serial pb2roscpp
```
