# ROS 2 arm workspace

ROS 2 **Lyrical**, **Gazebo Jetty**, and **MoveIt 2** on an Apple Silicon Mac.
The default environment is Ubuntu 26.04 in a native `linux/arm64` Docker
container. Gazebo runs **headless**. **Lichtblick** connects through Foxglove
Bridge at **`ws://localhost:8765`**, from the web build that the stack serves
at **`http://localhost:8080`** or from the desktop application.
The default robot is a **UFACTORY xArm6** with the **xArm Gripper** and a
wrist-mounted **RealSense D435i**, using the manufacturer's meshes,
kinematics, inertias, and joint limits. An overhead **D455** and two
**Ouster OS1-64** lidars watch the workcell. The original primitive arm is
also available.

## Quick start on this Mac

Clone the workspace with its registered dependencies:

```sh
git clone --recurse-submodules https://github.com/jcfurey/arm_ros2_ws.git
cd arm_ros2_ws
```

Install/start Docker Desktop, then run from this repository:

```sh
make image
make up
```

On first startup the container initializes any missing source submodules,
builds the workspace, and starts Gazebo, the controllers, MoveIt, two Ousters
at 10 Hz, both cameras, Foxglove Bridge, and Lichtblick. The first image and
C++ source build take several minutes.
Compilation defaults to one job for this 16 GB Mac; runtime CPU raycasting
uses two OpenMP threads per process.

Run `make lichtblick` to open Lichtblick in your browser, connected to the
bridge, with [`lichtblick/workcell.json`](lichtblick/workcell.json) as its
layout. The stack serves Lichtblick 1.30.0's web build at
`http://localhost:8080` (`LICHTBLICK_PORT`). The link loads the current
`workcell.json` each time and replaces the browser's layout named
`workcell`, so save changes you want to keep under another name. In the
desktop application (1.22 or newer), choose **File → Open connection →
Foxglove WebSocket**, enter `ws://localhost:8765`, and import the layout in
**Layouts → Import from file…**. The layout has four tabs:

- **Workcell:** the scene, both clouds, motion, pick, and gripper buttons,
  and motion status.
- **MoveIt:** a planning view with a dropdown for named poses, Cartesian
  tool targets, joint targets, and pick and place. It uses the local
  **Arm MoveIt** extension described below.
- **Cameras:** color and depth from the wrist and overhead cameras.
- **Plots:** the arm's joint positions and tracking error, and the wrist IMU
  with `WRIST_IMU=true`.

Only the open tab subscribes, and Gazebo renders a camera only while
something subscribes to it, so the Cameras tab is the one that costs
simulation time (see Simulation performance).

Install the MoveIt dropdown panel once per Lichtblick application/browser:

```sh
python3 scripts/package_moveit_panel.py --install  # installs in the desktop app's extensions folder
```

Reload the desktop app after installing, then import
[`lichtblick/workcell.json`](lichtblick/workcell.json). In the web app, open
**Extensions → Install local extension** and select
[`workspace.arm-moveit-0.1.1.foxe`](lichtblick/workspace.arm-moveit-0.1.1.foxe),
also served at
`http://localhost:8080/layouts/workspace.arm-moveit-0.1.1.foxe`, then reopen
`make lichtblick`. The package has no runtime or build dependencies. The
existing Workcell, Cameras, and Plots panels work without the extension.

In **MoveIt**, select a mode and target, click **Plan**, inspect the cyan tool
path and translucent goal robot, then click **Execute plan**. Planning does
not move the arm. The execute step checks the robot's start, the collision
scene, and every trajectory waypoint; moving the arm, changing speed, or
changing collision geometry clears the saved plan. A plan can execute once.
**Clear** removes a preview; **Stop** cancels the active motion and clears it.
Stop is sent whenever the panel is connected, even when the status display has
gone stale, and goes to MoveIt's trajectory executor as well as to the
workcell node.

A target is checked before the planner runs: one out of the arm's reach, or
one the arm can only reach in collision, fails within a second with that
reason instead of after the planner's five. If MoveIt stops answering, the
operation ends by itself with "MoveIt has not answered" after 20 s (5 s after
Stop), and the panel accepts commands again. With `make perception` the live
occupancy map changes several times a second; it does not clear a saved plan,
because Execute checks every waypoint against the current map anyway.

**Cartesian tool pose** uses `world` coordinates in metres and roll/pitch/yaw
in degrees; **Use current pose** fills the fields from MoveIt's FK. This is a
pose goal: MoveIt may choose a curved path. **Joint targets** has degree inputs
and sliders with the robot's limits, plus **Use current joints**. Inputs and
dropdown choices survive tab changes. **Pick and place** selects the part and
either its drop zone, its original position, or the other location. **Pick +
place** executes the whole sequence and shows its active step; Open/Close
operate the gripper. The shared speed control affects subsequent motions;
grasp approach/lift stays at 10% or slower. Execute/Clear/Stop stay visible
while scrolling the controls.

The clouds are colored by calibrated reflectivity on Ouster's fixed 0-100
scale (diffuse reflectance in percent; retroreflectors take the top color).
Without fixed **Value min/max** settings, Lichtblick rescales each cloud's
colors to that scan's own extremes, so a static scene brightens and dims
from scan to scan; set them when coloring by intensity or ambient too.
Those two channels carry shot noise, while the simulated reflectivity has
none. Intensity follows the response measured on real Rev7 OS1 recordings:
the median point reads about 3,200 counts on the front sensor (2.2 m away)
and 1,800 on the rear (3.4 m), under the detector's saturation near 5,700,
and varies by 1-2% from scan to scan as on the real sensor (try 0-6000).
Ambient varies by about ±10%.

For a custom **3D** panel, set display frame **`world`** and **Scene → Mesh up
axis → Z-up**, then reload Lichtblick (Cmd-R) so the mesh loader uses the new
axis. The manufacturer's STL meshes use ROS Z-up; the application's default
Y-up rotates each link mesh independently. Enable these topics:

- `/scene/markers`: SDF tables, fixtures, and obstacles with their colors.
- `/robot/markers`: xArm6 mesh materials and the Ouster sensor bodies.
- `/sensor/lidar/ouster_front/points` and `/sensor/lidar/ouster_rear/points`:
  the two independently decoded Ouster point clouds.
- `/tf` and `/tf_static`: robot and sensor transforms, and `parts/<model>`
  frames that follow the movable parts in Gazebo.

The cameras publish realsense-ros style topics under `/camera/wrist` and
`/camera/overhead`:

- `color/image_raw` and `color/camera_info`.
- `depth/image_rect_raw`: 16-bit millimetres with 0 for no depth, as
  realsense-ros publishes, including the D4xx stereo error. Its RMS grows as
  z^2 x 0.08 px / (focal length x baseline): about 4.7 mm at 1 m on the wrist
  D435i (50 mm baseline) and 2.5 mm on the overhead D455 (95 mm).
  `arm_bringup/realsense_depth` adds it to Gazebo's ideal depth.
- `depth/image_meters`: that ideal depth, as 32-bit float metres, for ground
  truth. `depth/points` is computed from it.
- `depth/camera_info`.

Each imager has its own field of view and optical frame, placed as in
realsense-ros's models: the D435i's color imager is 15 mm from its depth
imager, the D455's 59 mm. Both cameras run at 15 Hz, a native mode of each,
and render together. The wrist D435i sits on UFACTORY's camera stand with
its frame names; the overhead D455 is 2.1 m above the bench. Gazebo renders a
camera only while something subscribes to it, so unwatched cameras cost no
simulation time. `make camera-check` compares each depth stream with its
ideal depth and the error model.

The wrist D435i's IMU is off by default; `WRIST_IMU=true make up` (or
`wrist_imu:=true`) adds `/camera/wrist/imu`, a `sensor_msgs/Imu` at 200 Hz
in `camera_imu_optical_frame` like realsense-ros's unified IMU topic, without
orientation. It carries the BMI055's noise and a turn-on bias drawn per run,
as an uncalibrated unit would; `IMU_NOISE=false` removes both. Bullet
Featherstone, which the gripper needs, gives Gazebo's own IMU sensor no link
rotation rate or acceleration (its gyro read only noise while the wrist
turned at 0.4 rad/s), so `arm_bringup::PoseImu` takes its place and measures
the sensor's motion from its pose each step. `make imu-check` sweeps the arm
and compares the IMU with the motion in TF: with noise, after the bias
estimated at rest, the gyro was within 0.001-0.003 rad/s RMS of the 0.125
rad/s reference, and the wrist's own acceleration (0.03 m/s^2 RMS beside
gravity) matched with correlation 0.95-0.97. The simulated arm's velocity
changes in 10 ms steps, since gz_ros2_control holds each command for a
controller period, so the IMU averages acceleration over 10 ms; a real arm's
servos move more smoothly. The IMU costs 1-2% of a core in Gazebo, and the
bridge another 3% while something subscribes.

Use `/robot/markers` for the robot visuals; leave the additional
`/robot_description` display disabled to avoid overlapping copies.

The bridge serves the robot's mesh assets directly from the container; you
do not need to copy the meshes onto macOS. The host port is bound to loopback.
To use another port, run `FOXGLOVE_PORT=8766 make up` and connect to that port.

For motion, add a **Publish** panel for `/motion/command` with schema
`std_msgs/msg/String` and publish `{"data":"ready"}`, `{"data":"sweep"}`, or
`{"data":"home"}`. `{"data":"open"}` and `{"data":"close"}` move the gripper.
`{"data":"pick_cube"}` or `{"data":"pick_cylinder"}` picks that part and sets
it down at its `<part>_drop` frame, or back at its starting pose if it is
already there. `{"data":"stop"}` requests cancellation. Watch
`/motion/status` in a Raw Messages panel for planning, execution, the current
pick step, and results. Each command goes through MoveIt and its collision
scene.

```sh
make logs       # startup progress and ROS logs; Ctrl-C leaves the service running
make shell      # a ROS-sourced shell in the running container
make down       # stop the demo; keep build artifacts
```

## Other launch modes

```sh
make mock       # MoveIt + mock hardware + bridge, without Gazebo or sensor data
make headless   # same headless stack as make up
make demo       # use the original primitive arm
make up         # default headless xArm6 workcell with two Ousters
make obstacles  # use the obstacle-course world
make perception # also integrate both point clouds into MoveIt's occupancy map
ENABLE_CAMERAS=false make up  # no wrist or overhead camera
ENABLE_GRIPPER=false make up  # bare xArm6 flange, link_eef as the tip
WRIST_IMU=true make up        # also the wrist D435i's IMU on /camera/wrist/imu
```

These commands update the same Compose service, replacing the previous mode.
To select the primitive robot for any mode or test, set `ROBOT_MODEL=demo`:

```sh
ROBOT_MODEL=demo make mock
ROBOT_MODEL=demo make smoke-mock
```

On an Intel Linux machine, use `DOCKER_PLATFORM=linux/amd64 make image up`.
The Docker base tag is distribution-specific but follows upstream updates;
set `ROS_IMAGE` to an image digest if you need an immutable base.

## Motion test worlds

`workcell` is the default: a metal workbench, rear screen, fixture, and two
free parts to pick (a 40 mm cube and a 40 x 80 mm cylinder), with the xArm6
mounted on the bench. `obstacle_course` adds a
blocking post and guard wall. `empty` remains available for isolated robot
checks:

```sh
TEST_WORLD=workcell make up
TEST_WORLD=obstacle_course make up
TEST_WORLD=empty make up
make smoke-obstacles
```

The standalone SDF files live in `src/arm_bringup/worlds`. A `robot_mount` SDF
frame specifies the robot base position in both Gazebo and TF. The startup
scene loader reads each SDF collision's dimensions and model/link/collision
poses and applies those same obstacles and colors to MoveIt. Lichtblick's
world markers come from the same SDF source. Mock mode uses the same collision
scene.

The adapter supports static boxes, cylinders, spheres, and planes. It rejects
unsupported mesh/heightmap collisions, nested/includes, articulated dynamic
models, and external pose references so an unsupported obstacle cannot
silently disappear from planning. Static obstacles are a startup snapshot.
A non-static model with a single link is a movable part: `gz_relay`
publishes its Gazebo pose on `/parts/<model>/pose`, and the workcell UI moves
its MoveIt object to match and broadcasts its TF frame.

A pick is a top-down grasp across two opposite faces, planned and executed
step by step through MoveIt: open, approach above the part, a Cartesian
descent, grasp, attach the part to `link_tcp` in the planning scene, lift,
carry, lower, release, detach, and retreat. Of the two hand yaws that close
on those faces, only those whose lowered pose has a collision-free IK
solution are offered to the planner (the wrist camera sticks out 8 cm and
can meet the fixture on the way down). Only the finger pads may touch a
free part in MoveIt. The gripper is commanded to the part's width less 8 mm
and stalls on it at its rated force, as a real xArm Gripper is driven. Once
it has closed on the part, Gazebo welds the part to the hand
(`arm_bringup::GraspWeld`) until just before release, as grasp-fix plugins
do: Bullet touches a flat pad to a curved part at one point per step, and a
cylinder held by friction alone tipped in the hand and fell from about one
carry in four. With the weld, parts land within 1-2 mm of their target.
`weld_grasps:=false` holds parts by contact friction only. If a pick fails
while holding a part, `open` releases it.

For xArm6, `ready`, `sweep`, and `home` are the free test poses. In
`obstacle_course`, `blocked` intentionally intersects the red post and planning
to it must fail. The smoke check verifies loaded collision geometry, executes
the free sequence, confirms the measured joints, and checks rejection of the
blocked goal without executing it.

## Verify planning and execution

`make test` runs everything that needs no running stack: the scene, pick and
motion-panel unit tests and the bootstrap tests (which use disposable local
Git repositories) in the dev image, a check that the packaged `.foxe` matches
its source, and the panel's interaction tests in a Node image. Inside any
sourced ROS with `moveit_msgs`, `bash scripts/test.sh` runs the first three.
CI (`.github/workflows/ci.yaml`) runs these on pushes to `main` and `claude/**`
and on pull requests, then builds the image and runs the mock smoke test and the Gazebo smoke test,
the latter once more with the occupancy map enabled.

```sh
make test          # unit tests, packaged panel, panel interaction tests
make smoke-mock
make smoke
make smoke-obstacles
make bridge-check
make registration-check
make rates-check   # cloud rates, scan completeness, real-time factor
make pick-check    # picks both parts in the running workcell (moves them)
make imu-check     # with WRIST_IMU=true: wrist IMU against the arm's motion (moves the arm)
make lidar-motion-check  # a box moving past both Ousters (adds and removes it)
make moveit-check  # plan/preview/execute, tool/joint targets, scene changes, and Stop (moves the arm)
make perception-check  # with `make perception`: the occupancy map sees and forgets an unknown box (adds and removes it)
```

The three `smoke` commands start separate, temporary headless stacks with their own ROS
domain. It checks that both controllers are active and all six joint states
arrive, asks MoveIt to plan **and execute** `ready`, `sweep` (xArm6), and `home`, and checks the
measured joint positions, and closes and opens the gripper. Each also runs
the unit tests after building, and `moveit-check` against its own stack. The Gazebo
check also requires an advancing `/clock`, and in the workcell it picks the
cube and checks in Gazebo that it landed upright at its drop frame.
`ENABLE_PERCEPTION=true make smoke` adds `perception-check`.
The test stops its stack when complete. It shares the build volumes with the
main service, so run builds and smoke checks sequentially.

`bridge-check` checks the running service's WebSocket schemas, ROS streams,
and remote mesh assets. `registration-check` transforms each live cloud into
`world` and checks its distances to the actual SDF surfaces, catching rotated
or displaced scans even when topics and TF names look valid. The Gazebo smoke
checks include this registration check when Ousters are enabled.
`lidar-motion-check` drives a box at 1 m/s past both Ousters and compares
where each scan saw it with its pose at each point's timestamp.
`moveit-check` verifies plan-only operation, the preview markers, single-use
execution, collision-scene invalidation, input validation, Cartesian and joint
targets, that an unreachable tool target is refused at once with its reason,
cancellation before the target is reached, and the existing named
motions. It briefly adds and removes a far-away planning obstacle and returns
the arm home. `perception-check` requires both filtered clouds to arrive and a
preview to survive the changing map, then spawns a 0.3 m box that MoveIt has
no collision object for, requires the map to fill it, removes it, and requires
the map to clear. To test an explicit pick destination:

```sh
docker compose exec --user ros dev /usr/local/bin/ros-shell.sh \
  python3 scripts/check_pick.py --part pick_cube --destination drop
```

Panel interaction tests alone run with `cd lichtblick/moveit-panel && npm ci && npm test`;
`make moveit-panel` rebuilds the checked-in extension package from its source,
and `python3 scripts/package_moveit_panel.py --check` fails when the two differ.
The bridge can also verify a plan-only command over the same WebSocket used by
Lichtblick:

```sh
docker compose exec --user ros dev /usr/local/bin/ros-shell.sh \
  python3 scripts/check_bridge.py --plan ready
```

To test motion publishing through the bridge as well (moves the simulated arm):

```sh
docker compose exec --user ros dev /usr/local/bin/ros-shell.sh \
  python3 scripts/check_bridge.py --pose ready
```

To check an already-running demo (this moves the simulated arm):

```sh
make shell
ros2 run arm_bringup check_stack --robot xarm6 --mode gazebo --world workcell
# Add --ousters to verify live cloud returns and the sensor TF mounts too.
# Choose --robot demo or --mode mock to match the running stack.
```

## Development

```text
src/
  arm_description/     # URDF/xacro, inertias, limits, mock/Gazebo interfaces
  arm_moveit_config/   # SRDF, KDL IK, OMPL, controllers; xArm6 config in config/xarm6/
  arm_bringup/         # launch, SDF worlds, scene synchronization, integration check
  external/xarm_ros2/  # Git submodule; sparse checkout of xarm_description
  external/gz_sensors_ouster/  # cam_wip + ouster_sim_core submodule
  external/ouster-ros/        # matching pinned driver + ouster-sdk submodule
.gitmodules           # upstream URLs and tracking branches; Git records revisions
third_party/          # UFACTORY attribution and license
docker/               # ROS environment setup
scripts/              # bootstrap, build, demo, test, and smoke-test entry points
.github/workflows/    # CI: unit tests, panel tests, image build, smoke tests
```

Edit files on macOS. The repository is mounted at `/ws`; `build`, `install`, and
`log` use Docker named volumes, keeping Linux artifacts separate from native
macOS artifacts. The application runs as the container user `ros`.

After changes, stop the demo, rebuild, then start it again:

```sh
make down
make build
make up
```

When adding package dependencies, declare them in `package.xml`. Add binary
dependencies to the Dockerfile and run `make image` to persist them. For a
temporary development install, use a container shell:

```sh
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro lyrical -y
```

To use your own robot, replace the xacro and update the SRDF planning group,
collision exclusions, joint names, joint limits, controller lists, and IK
configuration together. `moveit_setup_assistant` is installed in the image.
Both robots use `world` as the fixed frame, planning group `arm`, and position
commands through `/arm_controller/follow_joint_trajectory`. The xArm6 uses
`link_base` as its base, `link_tcp` as its tip (`link_eef` without the
gripper), and joints `joint1` through `joint6`; the primitive arm uses
`base_link`, `tool0`, and `joint_1` through `joint_6`. The xArm6 gripper is
planning group `gripper` with `open` (85 mm) and `close` states, driven by
`drive_joint` through `/gripper_controller/gripper_cmd`
(`parallel_gripper_action_controller`).

The xArm6 wrapper is `arm_description/urdf/xarm6.urdf.xacro`. It loads UFACTORY's
model with limited joint ranges and selects either `mock_components/GenericSystem`
or `gz_ros2_control/GazeboSimSystem`. `xarm_gripper.xacro` adds the gripper
with box collisions fitted to the finger pads and knuckles, and keeps its
linkage as URDF mimic joints, which Bullet Featherstone enforces as
constraints. `cameras.xacro` defines the D435i and D455 sensors.
The robot's meshes are copies of UFACTORY's in `arm_description/meshes/sim`,
reduced to within 1 mm of the originals' surfaces: every camera image and
every MoveIt collision check processes them, and the originals carry ~130k
triangles. `scripts/simplify_meshes.py` regenerates them (it needs numpy,
scipy, and fast-simplification) and reports each mesh's deviation.
The local MoveIt configuration includes the
manufacturer's arm self-collision exclusions and the `home` and `ready` states.
Velocity and acceleration limits in our MoveIt configuration are conservative
simulation settings. The integration check reads named poses from the selected
SRDF, so it uses the same targets as the desktop motion-command interface.

`scripts/bootstrap.py` initializes only missing submodules and their missing
nested dependencies. Existing branches, commits, and uncommitted changes are
preserved, even when they differ from the workspace's recorded revisions.
It does not fetch updates or run a recursive checkout over existing work.
Builds after initialization work offline.

The UFACTORY submodule tracks `rolling`. Bootstrap configures a sparse checkout
of `xarm_description` if no sparse selection exists; `colcon.meta` supplies
that package's older CMake compatibility setting. Recursive clones also
initialize the manufacturer's C++ hardware SDK, which `build.sh` excludes
from simulation builds.

## Two Ouster viewpoints

The three repositories under `src/external` are registered Git submodules.
`gz_sensors_ouster` tracks `cam_wip` and includes its `ouster_sim_core`
submodule; `ouster-ros` includes the Ouster SDK. The workspace's Git links
record the integrated commits, and `.gitmodules` records their source URLs.

Develop directly in the Ouster checkouts. A normal recursive clone starts
submodules on detached HEADs; create a work branch before making commits:

```sh
git -C src/external/gz_sensors_ouster switch -c my-sensor-change
git -C src/external/gz_sensors_ouster/third_party/ouster_sim_core switch -c my-core-change
make build
```

Existing local branches, including your current `cam_wip` and core `main`,
are left as they are. Commit and push changes in each edited repository
first. For core changes, also commit and push the updated core Git link in
`gz_sensors_ouster`. Finally, commit the updated Git links in this workspace.
Use `git submodule status --recursive` to inspect the recorded checkouts.
Run `git submodule update --init --recursive` deliberately when you want to
restore the commits recorded by the parent repositories; builds do not run
that command against existing work.

`ouster_front_mount` and `ouster_rear_mount` SDF frames define the fixed sensor
poses. Both OS1-64s run in CPU `raycast` mode at 10 Hz, with different topic
namespaces. Native Ouster packets and metadata feed one `ouster_ros/os_cloud`
process per sensor, producing standard `PointCloud2` messages. No rendering
engine, CUDA, or GPU passthrough is needed for raycasting.

The `cam_wip` decoder requires distinct `sensor_frame` and `point_cloud_frame`
names. Clouds use `<name>/lidar_frame`, `sensor_frame` is `<name>/os_sensor`,
and `pub_static_tf` is false because the sensor URDF owns those transforms.
This prevents the decoder from applying the physical housing's additional
180° rotation and 36.18 mm offset to the simulation's identity XYZ lookup.

Clouds carry the exact simulation time of their scan's first column:
`os_cloud` runs with `timestamp_mode: TIME_FROM_INTERNAL_OSC`, and the
simulated sensor writes simulation time into its packets, as a PTP-synced
unit writes its synced clock. `TIME_FROM_ROS_TIME`, the usual mode for a
sensor without PTP, re-stamps each scan from its packets' arrival on
`os_cloud`'s clock. Here that put scans 6 ms early on average and single
scans up to 30 ms off, even with `os_cloud` on a 1 kHz clock. That error
matters wherever a cloud meets the moving arm's TF or another sensor's data.

Each point also shows moving geometry where it was at the point's own time,
the cloud stamp plus its `t` field. The sensor captures the arm and other
moving models 16 times per scan as it sweeps them, so each point sees them
within 4 ms of its time; one capture at the end of each scan had shown them
up to 100 ms late, 50 ms on average. `make lidar-motion-check` measures this.

```sh
LIDAR_HZ=2.0 make up           # lower the scan rate
ENABLE_OUSTERS=false make up  # robot/world only
make perception              # live dual-cloud occupancy integration
```

Occupancy mapping is opt-in. The known static SDF obstacles are always present
in MoveIt. With perception enabled, `sensors_3d.yaml` adds both clouds to a
4 cm octomap with robot self-filtering. MoveIt leaves out points that fall on
the robot or on a collision object it already has, so in the stock workcell,
where every surface is modelled, the map holds free space and no occupied
cells; it fills where the lidars see something MoveIt was not told about.
`make perception-check` proves that with a temporary box. Geometry that goes
away is cleared as the beams pass through where it was (the check's box clears
in a few seconds); restart the stack to drop the whole map.

The updater comes from `ros-lyrical-moveit-ros-perception`, which the image
installs. An image built before that was added runs `make perception` without
a map, because MoveIt only logs an error when the plugin is missing; rebuild
it with `make image`. The launch now refuses `perception:=true` when the
package is absent.

This integration simulates the xArm6 and gripper only, without a physical
robot driver. No robot IP address is used. Physical hardware support requires
the manufacturer's driver and a hardware-specific configuration.

The launch file starts MoveIt after controller activation; it starts the
controller spawner after Gazebo creates the robot. Gazebo's plugin owns the
controller manager in Gazebo mode. Mock mode starts `ros2_control_node` instead.

## Simulation performance

The defaults aim at a native `linux/arm64` container on an M1 Pro, with both
Ousters at 10 Hz in wall-clock time, not just sim time. Each item below was
measured to matter:

- **Physics:** Bullet Featherstone at 1 kHz. It enforces the gripper's mimic
  joints itself; DART cannot, and servoing them through gz_ros2_control cost
  18% of real time.
- **Pacing:** worlds set `real_time_factor` 0 and load
  `arm_bringup::RealtimePacer`, which paces steps against an absolute
  wall-clock schedule and repays late wake-ups. gz-sim's own throttle
  re-anchors at every late wake-up and held the container near 0.8.
- **PostUpdate systems in turn:** worlds set the `gz:policies`
  `parallel_postupdates` policy to false. By default gz-sim gives each system
  with a PostUpdate phase (sensors, controllers, scene broadcaster) its own
  thread and wakes them all every 1 ms step, and the wake-ups cost more than
  the systems' work: Gazebo used 130% of a core at real time where it now
  uses 114%.
- **Clock and part poses:** `gz_relay` republishes Gazebo's 1 kHz clock as
  `/clock` at 200 Hz (`clock_hz:=`) and part poses at 30 Hz, so ROS nodes do
  not wake for every physics step. The controller manager is the exception:
  it times each update by its node clock rather than the simulation time it
  is given, so on the 200 Hz clock its trajectory samples and joint-state
  stamps were off by 2.6 ms RMS (single updates by up to 18 ms). It receives
  every step on `/controller_manager/clock`, which brings that to 1.2 ms
  with 78% of updates exactly on time, for about 13% of a core.
- **Ouster raycasting:** the CPU raycaster balances its OpenMP threads over
  1024-ray rows (`gz_sensors_ouster` branch `perf/cpu-raycast-m1`), with
  `OMP_NUM_THREADS=2` and `OMP_WAIT_POLICY=PASSIVE` so idle threads sleep
  instead of spinning against the simulation thread.
- **Cloud delivery:** a 1024x64 cloud is a 3.1 MB message. Docker Desktop's
  VM caps socket receive buffers near 208 KB, and a container cannot raise
  `net.core.rmem_max`, so with default 1.3 KB fragments a best-effort
  subscriber loses most clouds. `docker/cyclonedds.xml` sends 64 KB
  fragments and the launch makes lidar packets and clouds reliable; a
  reliable subscriber receives every scan. Subscribe reliable in your own
  nodes.
- **Cameras:** separate color and depth sensors, bridged lazily and rendered
  only while subscribed. Docker on a Mac has no GPU, so Mesa's software
  rasterizer (llvmpipe) draws every image, and the settings are chosen for
  it: no MSAA on color images (`camera_anti_aliasing:=4` restores Gazebo's
  default), no shadow-casting lights, both cameras at 15 Hz so their images
  render together rather than interleaving, and robot meshes reduced to
  within 1 mm (`arm_description/meshes/sim`). Measured one at a time, MSAA
  and shadows were the largest costs.
- **No per-step extras:** no pose-publisher or altimeter systems, and the
  workcell UI asks MoveIt for the hand pose instead of decoding `/tf` and
  `/clock`.
- **Foxglove Bridge:** one executor thread (`num_threads: 1`). Its default
  of one thread per core woke every thread on each `/clock` message: about
  15% of a core with no client against 6% now, and 32% against 24% while
  streaming both clouds and both color images, all at their full rates.

Before the camera changes below, measured on this M1 Pro with Docker Desktop
(8 vCPUs, 8 GiB VM memory) and software OpenGL rendering:

| Configuration                        | Real-time factor | Ouster rate, sim / wall |
| ------------------------------------ | ---------------- | ----------------------- |
| Default (cameras present, unwatched) | 1.00             | 10.0 / 10.0 Hz          |
| Both color images subscribed         | ~0.25            | ~9.5-9.7 / 2.3-2.4 Hz   |
| All four color/depth images subscribed | ~0.23-0.27     | ~9.7-10.1 / 2.2-2.8 Hz  |

Camera rendering was the limiting workload. Measured one change at a time
on a 4-vCPU x86-64 VM, whose numbers matched the M1's above (0.28 with all
four images subscribed), each change's effect with all four images:

| Change (cumulative)                                   | Real-time factor |
| ----------------------------------------------------- | ---------------- |
| Before                                                | 0.28             |
| No shadow-casting light                               | 0.40             |
| No MSAA on color images                               | 0.67             |
| Overhead camera at 15 Hz, in step with the wrist      | 0.83             |
| Final: also RealSense depth and reduced meshes        | 0.76-0.78        |
| PostUpdate systems in turn (0.72-0.75 just before)    | 0.77-0.80        |

Back-to-back runs of one configuration differ by up to 0.04. In the final
configuration the ideal depth (`depth/image_meters`) measured 0.79-0.83, so
the realistic depth costs a few points. The 1 mm meshes' gain is within that
spread here (an earlier 31k-triangle reduction with ~1 cm errors raised one
run from 0.67 to 0.79); they also cut MoveIt's collision-checking work. With
nothing watched
the factor is 1.00, and one image stream alone costs 1-3%. Rendering alone
(Gazebo-level subscribers, no ROS) measured 0.84, so moving the images
through ROS costs the rest. These numbers need re-measuring on the M1. The
images keep their 15 Hz in simulation time; their wall-clock rate follows
the real-time factor.

`check_rates.py --cameras` subscribes to actual images to exercise rendering.
Gazebo can publish `CameraInfo` without producing image pixels, so measuring
metadata alone does not establish rendering performance. The checker also
accepts the old RealSense-style CameraInfo arguments and maps them to images.
For example, this measures all four streams and requires real-time Ousters:

```sh
docker compose exec --user ros dev /usr/local/bin/ros-shell.sh \
  python3 scripts/check_rates.py --min-hz 10 --min-rtf 0.97 --cameras \
  /camera/wrist/color/image_raw /camera/wrist/depth/image_rect_raw \
  /camera/overhead/color/image_raw /camera/overhead/depth/image_rect_raw
```

Disconnect image viewers to recover real-time operation (in the workcell
layout, leave the Cameras tab), or reduce the camera rates/resolutions when
editing the camera configuration. The camera
panels and depth processing share this rendering cost. See Gazebo's
[color](https://github.com/gazebosim/gz-sensors/blob/main/src/CameraSensor.cc)
and [depth](https://github.com/gazebosim/gz-sensors/blob/main/src/DepthCameraSensor.cc)
sensor implementations for the separate image and metadata update paths.

With images unwatched, the workcell scans contained about 12.5k and 11.5k
finite returns per sensor, and both clouds registered to the SDF surfaces
with a 4 mm median error.

## Native ROS installation

The existing installation at `~/ros2_lyrical/install/setup.zsh` can remain in
use for native macOS development. It currently provides RViz and xacro but
lacks the MoveIt/Gazebo/control packages required by this demo. The container
provides those dependencies without changing the native installation.

On an Ubuntu 26.04 Linux host or VM with ROS 2 Lyrical installed, the
workspace can run natively. Install MoveIt, Gazebo's control plugin, and the
controllers (these are already included in the Docker image):

```bash
sudo apt update
sudo apt install ros-lyrical-moveit ros-lyrical-moveit-configs-utils \
  ros-lyrical-moveit-ros-perception ros-lyrical-gz-ros2-control \
  ros-lyrical-ros2-controllers python3-websocket
```

Build only the workcell and its dependencies. This also works with a full
xArm checkout, without building its hardware drivers or Gazebo Classic
packages:

```bash
source /opt/ros/lyrical/setup.bash
python3 scripts/bootstrap.py
rosdep install --from-paths $(colcon list --packages-up-to arm_bringup --paths-only) \
  --ignore-src --rosdistro lyrical -y
CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4 colcon build --symlink-install \
  --packages-up-to arm_bringup --parallel-workers 2
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI="file://$PWD/docker/cyclonedds.xml"
export OMP_NUM_THREADS=2 OMP_WAIT_POLICY=PASSIVE
ros2 launch arm_bringup demo.launch.py
# Options: robot:=demo / mode:=mock / world:=obstacle_course / ousters:=false
```

With Lichtblick Desktop installed, install the MoveIt extension and open
the workcell layout connected to the native stack:

```bash
python3 scripts/package_moveit_panel.py --install
lichtblick --ds=foxglove-websocket --ds.url=ws://localhost:8765 \
  "$PWD/lichtblick/workcell.json"
```

After importing the layout once, omit the JSON argument to reuse the saved
layout. If Lichtblick was open when the extension was installed, restart it
to load the extension.

For the browser interface, start just the Lichtblick service in another
terminal with `docker compose up -d --no-deps lichtblick`, then run
`make lichtblick`. The ROS stack remains in the native terminal; Ctrl-C
stops it. Source `install/setup.bash` and use the same environment exports
in any terminal running ROS checks, such as
`ros2 run arm_bringup check_stack --ousters` or
`python3 scripts/check_bridge.py`.

Native host ROS nodes are not automatically connected to ROS nodes inside
Docker Desktop. Use `make shell` for CLI tools; all demo nodes use Cyclone DDS,
domain `42`, and localhost discovery inside the same container.

## Troubleshooting

- **Docker connection error:** start Docker Desktop and wait for its engine.
- **Lichtblick cannot connect:** inspect `make logs`, confirm Foxglove Bridge
  is listening, and check `FOXGLOVE_PORT` against the connection URL. The
  bridge accepts only the `foxglove.sdk.v1` WebSocket subprotocol, which
  Lichtblick offers from 1.22; older releases report "Connection failed".
- **Lichtblick alerts:** three "Definition of schema … has changed" errors
  (`visualization_msgs/Marker`, `sensor_msgs/PointField`,
  `lifecycle_msgs/msg/TransitionEvent`) are expected. Lichtblick preloads
  ROS 2 Humble's message definitions, and Lyrical's differ; panels still
  decode messages with the bridge's definitions. "High frequency topics"
  refers to `/tf` and the plotted controller state at 100 Hz. "Transform
  topics detected" suggests preloading transforms, which a live connection
  does not need.
- **Meshes missing:** the 3D panel needs the bridge's asset capability; use
  `make bridge-check` to test mesh retrieval and streamed ROS data.
- **Arm meshes look disconnected:** set **3D → Scene → Mesh up axis → Z-up**
  and reload Lichtblick. Keep only one robot visual display enabled.
- **Missing transforms after switching tabs:** confirm `/static_transforms`
  is running and `/tf_static` has one publisher. The launch combines the
  robot and sensor mounts from `/_tf_static_sources` into one complete
  retained message. Lichtblick can cache only the last message on a topic
  when a new panel opens, so separate partial snapshots can disconnect the
  sensors from `world` in the viewer. Rebuild/restart older stacks and reload
  Lichtblick once. `make bridge-check` verifies a complete snapshot.
- **Slow simulation:** run `make rates-check` for the real-time factor and
  sensor rates. Close camera views you are not using, or set
  `ENABLE_CAMERAS=false`; reduce `LIDAR_HZ`; give Docker Desktop more CPUs.
  Use `CMAKE_BUILD_PARALLEL_LEVEL=1` for builds on this 16 GB Mac.
- **Clouds arrive below their scan rate or with holes:** check that
  `CYCLONEDDS_URI` points at `docker/cyclonedds.xml` and that subscribers
  request reliable QoS (best-effort subscribers lose whole clouds; see
  Simulation performance).
- **No simulated motion:** Gazebo must be playing; the launch starts it with
  `-r`. Inspect `ros2 control list_controllers` in `make shell`.
- **Clean rebuild:** `make down`, then `docker compose down -v` removes the
  generated Docker build/install/log volumes. Your source files remain.

## Upstream references

- [ROS Docker images and supported architectures](https://hub.docker.com/_/ros)
- [Gazebo/ROS version pairings](https://gazebosim.org/docs/latest/ros_installation/)
- [MoveIt 2 binary installation](https://moveit.ai/install-moveit2/binary/)
- [Lyrical gz_ros2_control integration](https://control.ros.org/lyrical/doc/gz_ros2_control/doc/index.html)
- [UFACTORY xArm ROS 2 repository](https://github.com/xArm-Developer/xarm_ros2)
- [Pinned xArm model revision](https://github.com/xArm-Developer/xarm_ros2/tree/6d53d8b2eac2c630002ecad18fd5bc7c967a8b34)
- [Ouster simulation cam_wip branch](https://github.com/jcfurey/gz_sensors_ouster/tree/cam_wip)
- [Lichtblick live data connections](https://lichtblick-suite.github.io/docs/docs/connecting-to-data/live-data/)
- [Lichtblick releases and container image](https://github.com/lichtblick-suite/lichtblick/releases)
- [Foxglove ROS bridge](https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge)
