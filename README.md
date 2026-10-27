# ROS 2 Integration for Diana Robots

> **Migration notice:** _diana_ros2_ was migrated from
> [_franka_ros2_](https://github.com/frankarobotics/franka_ros2) to support the
> Diana series of robotic arms, which also communicate over **libfranka**. The
> package layout, code structure, and configuration style intentionally mirror
> _franka_ros2_, so the robot description, messages, hardware interface, semantic
> components, controllers, MoveIt configuration, and Gazebo bringup have been
> adapted for Diana robots while keeping the two repositories easy to compare.

> **Note:** _diana_ros2_ is not officially supported on Windows.

> ### ⚠️ IMPORTANT: libfranka is built for ONE Diana arm type
>
> The rate-limiting parameters in libfranka (joint/Cartesian/elbow jerk,
> acceleration and velocity limits) differ between Diana3, Diana7 and Diana7G2.
> As a **temporary solution**, they are selected **at compile time** through
> [diana_libfranka_overlay](https://github.com/AgileRobots-APAC/diana_libfranka_overlay),
> **not** by the `robot_type` launch argument.
>
> - Before building, run `src/scripts/select_diana_arm.sh <diana3|diana7|diana7g2>`
>   to apply the libfranka patch that matches your **real robot**
>   (see [Select the Diana arm for libfranka](#select-the-diana-arm-for-libfranka-static-build)).
> - **Every time you switch to a different real robot model, you must run the
>   script again and rebuild libfranka and every package that depends on it**
>   (see [Switch to a different real robot](#switch-to-a-different-real-robot)).
> - `robot_type` must match the arm libfranka was built for. Nothing checks this
>   at runtime, and a mismatch silently applies the wrong limits when rate
>   limiting is enabled.
> - One workspace build serves one arm type. Running different Diana models at
>   the same time (e.g. a multi-robot `diana.config.yaml` mixing `diana3` and
>   `diana7`) is **not supported** with this build.
>
> Simulation and fake hardware (`use_fake_hardware:=true`, Gazebo) do not use
> the libfranka rate limits, so they work with any build.

## Table of Contents

- [About](#about)
- [Packages](#packages)
- [Caution](#caution)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
  - [Select the Diana arm for libfranka (static build)](#select-the-diana-arm-for-libfranka-static-build)
  - [Switch to a different real robot](#switch-to-a-different-real-robot)
- [Test the Setup](#test-the-setup)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About

The **diana_ros2** repository provides a **ROS 2** integration of **libfranka**,
allowing efficient control of Diana series arms within the ROS 2 framework. It is
a migration of the Franka ROS 2 stack for the Diana robots, which share the same
7-DOF kinematic structure but expose the libfranka communication interface.

The project is designed to facilitate robotic research and development by
providing a robust interface for controlling the research versions of the Diana
robots. Currently, three arm types are supported:

| Robot type | Degrees of freedom | Payload |
|---|---|---|
| `diana3` | 7 | 3 kg |
| `diana7` | 7 | 7 kg |
| `diana7g2` | 7 | 7 kg |

Because Diana arms cannot be fitted with a Franka Hand, the gripper packages and
gripper-specific configurations inherited from _franka_ros2_ are not used. An
optional Diana end-effector (`ee_id:=diana_hand`) is provided instead, and it is
disabled by default.

For convenience, we provide `Dockerfile` and `docker-compose.yml` files. While it
is possible to build **diana_ros2** directly on your local machine, this approach
requires manual installation of certain dependencies, while many others will be
automatically installed by the **ROS 2** build system (e.g., via **rosdep**).
This can result in a large number of libraries being installed on your system,
potentially causing conflicts. Using Docker encapsulates these dependencies
within the container, minimizing such risks. Docker also ensures a consistent and
reproducible build environment across systems. For these reasons, we recommend
using Docker.

## Packages

The workspace mirrors the _franka_ros2_ structure:

| Package | Description |
|---|---|
| `diana_bringup` | Launch files and run-time configurations for using Diana robots with `ros2_control`. |
| `diana_description` | URDF/Xacro files, meshes, and RViz resources for the Diana robots. |
| `diana_msgs` | Messages and actions specific to Diana robots. |
| `diana_hardware` | Hardware interface plugins for using Diana robots with `ros2_control`. |
| `diana_semantic_components` | Semantic components for using Diana robots with `ros2_control`. |
| `diana_robot_state_broadcaster` | Controller that broadcasts Diana robot states. |
| `diana_example_controllers` | Example controllers for controlling Diana robots with `ros2_control`. |
| `diana_moveit_config` | MoveIt2 configuration files for the Diana robots. |
| `diana_gazebo` | Gazebo bringup (`diana_gazebo_bringup`) and Gazebo hardware interface (`diana_gazebo_hardware`). |
| `diana_gripper` | Diana gripper interface. |
| `diana_ros2` | Meta package that pulls in the whole stack. |
| `libfranka` | Cloned as a dependency through [diana_libfranka_overlay](https://github.com/AgileRobots-APAC/diana_libfranka_overlay) (see `dependency.repos`): the versioned overlay tag, together with one per-arm Diana patch, selects the upstream libfranka version and builds it for a single arm type. |

## Caution

This package is in rapid development. Users should expect breaking changes and
are encouraged to report any bugs via the project issue tracker.

## Setup

### Prerequisites

This repository contains a `.repos` file (`dependency.repos`) that helps you
clone the required dependencies for diana_ros2.

_**diana_ros2**_ is built upon _**ROS 2 humble**_.

To set up your ROS 2 environment, follow the official _**humble**_ installation
instructions provided
[**here**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
The guide discusses two main installation options: **Desktop** and **Bare Bones**.

#### Choose **one** of the following:

- **ROS 2 "Desktop Install"** (`ros-humble-desktop`)
  Includes a full ROS 2 installation with GUI tools and visualization packages
  (e.g., RViz and Gazebo).
  **Recommended** for users who need simulation or visualization capabilities.

- **"ROS-Base Install (Bare Bones)"** (`ros-humble-ros-base`)
  A minimal installation that includes only the core ROS 2 libraries.
  Suitable for resource-constrained environments or headless systems.

```bash
# replace <YOUR CHOICE> with either ros-humble-desktop or ros-humble-ros-base
sudo apt install <YOUR CHOICE>
```

---

Also install the **Development Tools** package:

```bash
sudo apt install ros-dev-tools
```

Installing the **Desktop** or **Bare Bones** should automatically source the
**ROS 2** environment but, under some circumstances you may need to do this
again:

```bash
source /opt/ros/humble/setup.sh
```

### Local Machine Installation

1. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/diana_ros2_ws/src
   cd ~/diana_ros2_ws  # not into src
   ```

2. **Clone the Repository:**
   ```bash
   git clone https://github.com/AgileRobots-APAC/diana_ros2.git src
   ```

3. **Install the dependencies:**
   ```bash
   vcs import src < src/dependency.repos --recursive --skip-existing
   ```
   `--recursive` is required: it also clones the `libfranka` submodule of
   `diana_libfranka_overlay`.

4. **Select your Diana arm and patch libfranka (required for real robots):**
   ```bash
   # replace <ARM> with the arm of your real robot: diana3, diana7 or diana7g2
   src/scripts/select_diana_arm.sh <ARM>
   ```
   The script must run before the first build. Without it, libfranka is built
   unpatched, with the upstream Franka FR3 limits. See
   [Select the Diana arm for libfranka](#select-the-diana-arm-for-libfranka-static-build).

5. **Detect and install project dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```

6. **Build:**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
   ```

7. **Adjust Environment:**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```

### Docker Container Installation

The **diana_ros2** package includes a `Dockerfile` and a `docker-compose.yml`,
which allows you to use `diana_ros2` packages without manually installing
**ROS 2**. Support for Dev Containers in Visual Studio Code is also provided.

For detailed instructions on preparing VS Code to use the `.devcontainer`, follow
the setup guide from
[VS Code devcontainer_setup](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Clone the Repository:**
   ```bash
   git clone http://10.11.0.11/agileapac/dianaros2/diana_ros2.git
   cd diana_ros2
   ```

   We provide separate instructions for using Docker with Visual Studio Code or
   the command line. Choose one of the following options:

   Option A: Set up and use Docker from the command line (without Visual Studio Code).

   Option B: Set up and use Docker with Visual Studio Code's Docker support.

#### Option A: using Docker Compose

2. **Save the current user id and container name into a file:**
   ```bash
   echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)\nCONTAINER_NAME=diana_ros2" > .env
   ```
   It is needed to mount the folder from inside the Docker container.

3. **Build the container:**
   ```bash
   docker compose build
   ```

4. **Run the container:**
   ```bash
   docker compose up -d
   ```

5. **Open a shell inside the container:**
   ```bash
   docker exec -it diana_ros2 /bin/bash
   ```
   The container entrypoint already runs `vcs import` to clone the latest
   dependencies into `/ros2_ws/src`.

6. **Select your Diana arm and patch libfranka (required for real robots):**

   Inside the container, from `/ros2_ws`:
   ```bash
   # replace <ARM> with the arm of your real robot: diana3, diana7 or diana7g2
   src/scripts/select_diana_arm.sh <ARM>
   ```
   See [Select the Diana arm for libfranka](#select-the-diana-arm-for-libfranka-static-build).

7. **Build the workspace:**
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

8. **Source the built workspace:**
   ```bash
   source install/setup.bash
   ```

9. **When you are done, you can exit the shell and delete the container:**
   ```bash
   docker compose down -t 0
   ```

#### Option B: using Dev Containers in Visual Studio Code

2. **Open Visual Studio Code ...**

   Then, open the folder `diana_ros2`.

3. **Choose `Reopen in container` when prompted.**

   The container will be built automatically, as required.

4. **Select your Diana arm and patch libfranka (required for real robots):**

   Open a terminal in `/ros2_ws` and run:
   ```bash
   # replace <ARM> with the arm of your real robot: diana3, diana7 or diana7g2
   src/scripts/select_diana_arm.sh <ARM>
   ```
   See [Select the Diana arm for libfranka](#select-the-diana-arm-for-libfranka-static-build).

5. **Build the workspace:**
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

6. **Source the built workspace environment:**
   ```bash
   source install/setup.bash
   ```

### Select the Diana arm for libfranka (static build)

> ⚠️ **The arm type is fixed when libfranka is compiled.** Build for the arm you
> will actually connect to, and rebuild whenever you switch arms (see
> [Switch to a different real robot](#switch-to-a-different-real-robot)).

`dependency.repos` clones
[diana_libfranka_overlay](https://github.com/AgileRobots-APAC/diana_libfranka_overlay)
to `src/diana_libfranka_overlay` and pins it to a versioned tag. The tag encodes
both the overlay release and the upstream libfranka version, and ends with the
target arm. The script reads the exact tag from `dependency.repos` and replaces
only its arm suffix. Updating `dependency.repos` is therefore sufficient when
the overlay or libfranka version changes.

The overlay contains upstream libfranka as a git submodule
(`src/diana_libfranka_overlay/libfranka`, the package colcon builds) and one
patch per arm. The patch replaces `include/franka/rate_limiting.h` with the
limits for that arm.

For example, if `dependency.repos` pins
`v1.2.0-libfranka0.22.0-diana3`, selecting `diana7` checks out
`v1.2.0-libfranka0.22.0-diana7`.

| Real robot | `robot_type` / script argument | Overlay tag | Patch |
|---|---|---|---|
| Diana3 | `diana3` | `<release>-libfranka<libfranka-version>-diana3` | `patches/diana3.patch` |
| Diana7 | `diana7` | `<release>-libfranka<libfranka-version>-diana7` | `patches/diana7.patch` |
| Diana7G2 | `diana7g2` | `<release>-libfranka<libfranka-version>-diana7g2` | `patches/diana7g2.patch` |

Use `scripts/select_diana_arm.sh` to select the arm. Run it from the workspace
root (`~/diana_ros2_ws` locally, `/ros2_ws` in Docker):

```bash
src/scripts/select_diana_arm.sh diana3     # or diana7 / diana7g2
```

The script:

1. removes the previously applied arm patch from the libfranka submodule;
2. reads the overlay tag from `dependency.repos`, replaces its arm suffix with
   the requested arm, and checks out that tag and its pinned libfranka commit;
3. applies `patches/<arm>.patch`;
4. checks that `rate_limiting.h` now holds the limits for `<arm>`, and prints
   the rebuild command.

It does not delete your own work. If the libfranka submodule or the overlay
has changes that are not a Diana arm patch, it stops and lists them. Save them,
then re-run with `--force` to discard the libfranka changes.

Other useful options (see `src/scripts/select_diana_arm.sh --help`):

```bash
src/scripts/select_diana_arm.sh --status            # show which arm libfranka is patched for
src/scripts/select_diana_arm.sh diana7 --no-fetch   # skip 'git fetch' (offline)
```

After the script, the libfranka submodule shows as modified in `git status`.
This is expected. Continue with the build step of your installation method.

To check the arm that is actually **installed** after building:

```bash
grep -n "kMaxTranslationalJerk =" install/libfranka/include/franka/rate_limiting.h
```

| Value | Arm |
|---|---|
| `2500` | Diana3 |
| `3000.0` | Diana7 |
| `5000.0` | Diana7G2 |
| `4500.0` | **Not patched** (upstream Franka FR3 limits). Do not use with a Diana robot |

If an older checkout left a separate `src/libfranka` in your workspace, remove
it. Otherwise colcon finds two packages named `libfranka` and stops with a
duplicate-package error. The script warns about this.

<details>
<summary>Manual steps (what the script does)</summary>

Run from the workspace root, with `ARM` set to `diana3`, `diana7` or `diana7g2`.
The `reset`/`clean` commands delete **all** local changes and untracked files in
the libfranka submodule.

```bash
ARM=diana3
OVERLAY_TAG="$(
  awk '
    $1 == "diana_libfranka_overlay:" { in_overlay = 1; next }
    in_overlay && $1 == "version:" { print $2; exit }
    in_overlay && /^  [^ ]/ { exit }
  ' src/dependency.repos
)"
OVERLAY_TAG="${OVERLAY_TAG%-*}-${ARM}"
git -C src/diana_libfranka_overlay/libfranka reset --hard
git -C src/diana_libfranka_overlay/libfranka clean -fd
cd src/diana_libfranka_overlay
git fetch --tags
git switch --detach "${OVERLAY_TAG}"
git submodule update --init --recursive
git -C libfranka apply --check ../patches/${ARM}.patch
git -C libfranka apply ../patches/${ARM}.patch
cd ../..
```

The patch path is relative to the `libfranka` directory because of `-C`.

</details>

### Switch to a different real robot

> ⚠️ **You must recompile when you connect to a different Diana model.**
> Changing `robot_type` alone does **not** change the rate limits compiled into
> libfranka.

Run these commands from the workspace root:

1. **Select the new arm:**
   ```bash
   # replace <ARM> with the new arm: diana3, diana7 or diana7g2
   src/scripts/select_diana_arm.sh <ARM>
   ```

2. **Rebuild libfranka and every package that depends on it.** Packages such as
   `diana_hardware` compile the rate-limit constants from the libfranka headers
   into their own binaries, so rebuilding only libfranka is not enough:
   ```bash
   colcon build --symlink-install --packages-above libfranka --cmake-clean-first \
     --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
   ```
   If you are not sure the rebuild picked up the change, do a full clean build
   instead:
   ```bash
   rm -rf build install log
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
   ```

3. **Source the workspace again and launch with the matching `robot_type`:**
   ```bash
   source install/setup.bash
   ```

## Test the Setup

### Run a sample ROS 2 application

To verify that your setup works correctly without a robot, you can run the
following command to use dummy hardware:

```bash
ros2 launch diana_moveit_config moveit.launch.py \
  robot_type:=diana3 robot_ip:=dont-care use_fake_hardware:=true
```

Use `robot_type` to select one of `diana3`, `diana7`, or `diana7g2`. You can use
the argument `load_gripper` to activate or deactivate the end-effector and
`ee_id` to set which end-effector you want to use. By default, no gripper is
loaded.

If you want to run this example with namespaces, use the `namespace` argument and
manually set your namespace in the corresponding `moveit_<robot_type>.rviz` file
under `Move Group Namespace`.

### Run a ROS 2 example controller

To run any example controller, make sure to add your desired configuration in
`diana.config.yaml` and run:

```bash
ros2 launch diana_bringup example.launch.py controller_names:=your_desired_controller
```

You can select one of the controllers from `controllers.yaml`.

> Note: elbow and cartesian examples call services (e.g.
> `set_full_collision_behavior`) and require interfaces that are only provided by
> the real `diana_hardware` interface, so they need `use_fake_hardware: "false"`
> and a reachable robot. With `mock_components/GenericSystem` use the joint-level
> examples (`gravity_compensation_example_controller`,
> `joint_impedance_example_controller`).

### Run different controllers for different robots

If you want to run a specific controller for each robot, you must specify the
controllers you want to run as follows (example for three robots):

```bash
ros2 launch diana_bringup example.launch.py controller_names:="joint_position_example_controller,gravity_compensation_example_controller,joint_impedance_example_controller"
```

If fewer controllers than the number of robots are specified, only the first
controller is used for all the robots.

### Run multiple robots with namespaces

To bring up more than one Diana robot, add an entry per robot in
`diana_bringup/config/diana.config.yaml`, giving each one a unique `namespace`,
`robot_type`, and `robot_ip`. Then run:

```bash
ros2 launch diana_bringup example.launch.py
```

Each robot's topics and services are namespaced, e.g. the controller manager can
be reached under `/NS_1/controller_manager`. You can inspect the result with:

```bash
ros2 topic list | grep joint_states
ros2 service list | grep controller_manager
```

### Run Gazebo examples with ROS 2

To visualize the Diana robot model:

```bash
ros2 launch diana_gazebo_bringup visualize_diana_robot.launch.py robot_type:=diana3
```

To run a Diana arm with an example controller inside Gazebo:

```bash
ros2 launch diana_gazebo_bringup gazebo_diana_arm_example_controller.launch.py \
  robot_type:=diana3 controller:=gravity_compensation_example_controller
```

## Test the build

```bash
colcon test
```

> Remember, diana_ros2 is under development.
> Warnings can be expected.

## Troubleshooting

### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot,
avoid using Docker Desktop. It may not provide the necessary real-time
capabilities required for reliable communication with the robot. Instead, using
Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent
timeout issues. For guidance on setting up a real-time kernel, please refer to
the libfranka documentation.

### Gazebo or visualization does not start

Make sure both the ROS 2 environment and the built workspace are sourced:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then confirm the required packages are discoverable:

```bash
ros2 pkg list | grep diana
```

## Contributing

Contributions are welcome! Please see `CONTRIBUTING.md` for more details on how
to contribute to this project.

## License

All packages of diana_ros2 are licensed under the Apache 2.0 license.

## Contact

For questions or support, please open an issue on the project issue tracker.

See the [Franka Control Interface (FCI) documentation](https://frankarobotics.github.io/docs)
for more information about libfranka.
