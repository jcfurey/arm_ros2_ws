The official xArm robot description is fetched from
[UFACTORY's xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) at the revision
in `dependencies/xarm.json`. The checkout and its original license remain in
`src/external/xarm_ros2`; generated and external sources are excluded from this
repository's commits.

The arm collision exclusions in
`src/arm_moveit_config/config/xarm6/arm.srdf` are adapted from
`xarm_moveit_config/srdf/_xarm6_macro.srdf.xacro` at the same revision, under the
license in this directory. Our SRDF uses the planning group name `arm` and adds
the simulation `ready` state.
