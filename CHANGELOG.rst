^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for arm_ros2_ws
^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
Changes on top of ``main`` at 99e56e2 (2026-10-24). Figures were measured in Docker on a 4-vCPU
x86-64 VM unless noted.

**Simulation performance**

* Both Ousters run at 10 Hz in real time: the real-time factor is 0.99-1.01 with both at 10 Hz,
  from 0.72 with lossy clouds.

  - Cyclone DDS uses 64 KB fragments (``docker/cyclonedds.xml``), and lidar packets and clouds
    are reliable. A 3.1 MB cloud in 1.3 KB fragments overran Docker Desktop's socket buffers.
  - ``arm_bringup::RealtimePacer`` paces steps against an absolute wall-clock schedule (the
    worlds set ``real_time_factor`` 0); gz-sim's own throttle held the container near 0.8.
  - ``gz_relay`` republishes Gazebo's 1 kHz clock as ``/clock`` at 200 Hz and part poses from
    ``dynamic_pose/info``, instead of per-step ROS wake-ups and per-part pose publishers.
  - Bullet Featherstone physics enforces the gripper's mimic joints natively.
  - ``OMP_WAIT_POLICY=PASSIVE``, and the workcell UI asks MoveIt for the hand pose instead of
    decoding ``/tf``.

* Gazebo's PostUpdate systems run in turn on the simulation thread
  (``gz:policies/parallel_postupdates`` false): Gazebo uses 114% of a core at real time instead
  of 130%.
* RealSense rendering: no shadow-casting lights, no MSAA on color images
  (``camera_anti_aliasing:=4`` restores it), the overhead D455 at 15 Hz, and robot meshes within
  1 mm of UFACTORY's (130k to 53k triangles). With all four images watched the real-time factor
  is 0.76-0.78 instead of 0.25-0.28, and 1.00 with none watched.
* Foxglove Bridge runs one executor thread: 6% of a core with no client instead of 15%.

**Manipulation and sensing**

* Add the UFACTORY xArm Gripper: manufacturer kinematics and meshes, its mimic linkage, a
  parallel gripper action controller, and a MoveIt group with open (85 mm) and close states.
* Add a RealSense D435i on the wrist and an overhead D455, as separate color and depth sensors
  with realsense-ros topics, bridged only while watched.
* Movable parts are mirrored into MoveIt from Gazebo's poses. ``/motion/command`` ``pick_cube``
  and ``pick_cylinder`` run a top-down pick and place, and ``arm_bringup::GraspWeld`` holds a
  stalled grasp until release (``weld_grasps:=false`` holds by friction only).
* ``depth/image_rect_raw`` is what a D4xx publishes: 16UC1 millimetres, 0 for no depth, with
  stereo error of RMS ``z^2 * 0.08 / (focal_px * baseline)``. The ideal depth moves to
  ``depth/image_meters``, and the D455's frames follow realsense-ros.
* ``WRIST_IMU=true`` adds the D435i IMU on ``/camera/wrist/imu`` at 200 Hz with BMI055 noise and
  a per-run turn-on bias (``IMU_NOISE=false`` removes both). ``arm_bringup::PoseImu`` derives it
  from the sensor's motion, because Bullet Featherstone reports no link velocities.
* xarm_ros2 tracks its rolling branch (6d53d8b).

**Ouster lidars**

* Intensity follows the response measured on real Rev7 recordings: the front OS1-64 reads a
  median 3,240 counts at 2.2 m with 1.25% scan-to-scan noise and the rear 1,850 at 3.4 m with
  2.4%, where the old model gave about 44 with 15%.
* Retroreflectors encode on Ouster's calibrated reflectivity scale, where byte 255 is 864x a
  white target, and ``base_reflectivity`` reads back as the byte it names (53 and 59 read one
  low). Neither changes the workcell, which has no retroreflectors and keeps the default 50.
* [BUGFIX]: clouds are stamped from their packets' simulation time (``TIME_FROM_INTERNAL_OSC``).
  Stamped on arrival, they were 6.2 ms early on average and up to 30 ms off.
* Moving geometry appears at each point's own time: a mean lag of 0.4-0.6 ms, from 46-55 ms
  (``make lidar-motion-check``).
* The CPU raycaster balances its threads by beam row.

**Controllers and launch**

* [BUGFIX]: slow Gazebo starts spawned duplicate robots: ``-allow_renaming false`` read as
  ``true``, so every spawn retry added a renamed copy with its own controller manager.
* The controller manager receives every simulation clock message on
  ``/controller_manager/clock``, so 78% of its updates are exactly 10 ms apart; through the
  200 Hz ``/clock`` their spacing had a 3.7 ms standard deviation.
* Controller spawner timeouts are 60 s, past a cold host's first shader compile.

**Visualization**

* A Lichtblick 1.30.0 service runs at ``localhost:8080`` (``make lichtblick``), with the workcell
  layout in Workcell, Cameras and Plots tabs; only the open tab subscribes.
* [BUGFIX]: the clouds are colored by calibrated reflectivity on Ouster's fixed 0-100 scale.
  Scaled to each scan's own range, static objects brightened and dimmed from scan to scan.

**Checks**

* Add ``make rates-check``, ``pick-check``, ``camera-check``, ``imu-check`` and
  ``lidar-motion-check``, a pick in the Gazebo smoke test, and gripper checks in
  ``check_stack``.
* [BUGFIX]: camera rate checks subscribe to rendered images and reject missing streams and a
  stalled clock, and a pick check clears the previous pick's latched status.
