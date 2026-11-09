# Arm MoveIt panel

A local Lichtblick extension for the arm workcell. The dropdown selects named
poses, Cartesian tool poses, joint targets or pick and place. Targets are planned
without moving; a cyan tool path and translucent goal robot appear on
`/motion/preview`. Execute runs the saved trajectory after checking the current
start and every waypoint for collisions. Stop cancels the active operation; it
is sent to the workcell node and to MoveIt's trajectory executor whenever the
panel is connected, even if the status display has gone stale.

Unreachable and colliding targets are refused within a second, with the reason
in the status line. If MoveIt stops answering, the operation times out and the
panel accepts commands again. A live occupancy map (`make perception`) does not
clear a saved plan; Execute checks every waypoint against the current map.

Cartesian targets use metres and roll/pitch/yaw in degrees in `world`. Joint
targets use degrees in the panel and radians on ROS. Pick and place runs the
whole sequence; its destination can be the drop zone, original position or the
other location. The approach and lift stay at 10% speed or slower.

Run `python3 scripts/package_moveit_panel.py` from the workspace root to build
the `.foxe` file. There are no build dependencies. In Lichtblick's Extensions
sidebar, install `workspace.arm-moveit-0.1.1.foxe`, then import `workcell.json`.
The web app serves the package at
`http://localhost:8080/layouts/workspace.arm-moveit-0.1.1.foxe`.

For the desktop app, `python3 scripts/package_moveit_panel.py --install` also
installs the unpacked extension into `~/.lichtblick-suite/extensions`.
Reload Lichtblick and import the updated layout after installing.

The panel uses [Lichtblick's extension API](https://lichtblick-suite.github.io/docs/docs/extensions/extension-api/panel-extension-context/)
and its existing data connection; it
does not create a separate WebSocket connection. It subscribes only to the
low-rate motion status topic and releases that subscription when closed.
