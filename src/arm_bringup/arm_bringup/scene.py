"""Read static primitive SDF worlds into a planning scene, without ROS imports.

Supports the explicitly authored model/link/collision pose hierarchy in our
worlds. Unsupported geometry, includes, moving bodies, or frame references fail
instead of silently leaving obstacles out of MoveIt.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


IDENTITY = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def numbers(text, count):
    values = tuple(float(value) for value in text.split())
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Expected {count} finite values: {text!r}")
    return values


def quaternion(rpy):
    r, p, y = (angle / 2 for angle in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
            w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s)


def compose(parent, local):
    rotation = parent[3:]
    translated = multiply(multiply(rotation, (*local[:3], 0.0)),
                          (-rotation[0], -rotation[1], -rotation[2], rotation[3]))
    return tuple(parent[i] + translated[i] for i in range(3)) + multiply(rotation, local[3:])


def pose_values(element, allowed_frame=""):
    pose = element.find("pose")
    if pose is None:
        return (0.0,) * 6
    if pose.get("relative_to", "") not in ("", allowed_frame):
        raise ValueError("Scene adapter requires poses relative to their parent")
    if pose.get("degrees", "false") != "false" or pose.get("rotation_format", "euler_rpy") != "euler_rpy":
        raise ValueError("Scene adapter requires Euler RPY poses in radians")
    return numbers(pose.text or "", 6)


def local_pose(element):
    values = pose_values(element)
    return values[:3] + quaternion(values[3:])


@dataclass(frozen=True)
class SceneObject:
    id: str
    shape: str
    dimensions: tuple
    pose: tuple
    color: tuple


@dataclass(frozen=True)
class WorldScene:
    name: str
    mount: tuple
    objects: tuple
    frames: dict


def load_world(path):
    world = ET.parse(Path(path)).getroot().find("world")
    if world is None:
        raise ValueError("Expected a standalone SDF world")
    if world.findall("include"):
        raise ValueError("Inline world geometry is required for planning-scene synchronization")
    mount = (0.0,) * 6
    frames = {}
    for frame in world.findall("frame"):
        if frame.get("attached_to", "world") == "world":
            frames[frame.attrib["name"]] = pose_values(frame, "world")
        if frame.get("name") == "robot_mount":
            if frame.get("attached_to", "world") != "world":
                raise ValueError("robot_mount must attach directly to world")
            mount = pose_values(frame, "world")
    objects = []
    for model in world.findall("model"):
        if model.findall("model") or model.findall("include"):
            raise ValueError("Nested models/includes are not supported by the scene adapter")
        if model.findtext("static", "false").strip() not in ("true", "1"):
            raise ValueError(f"Moving model cannot be mirrored as a static obstacle: {model.get('name')}")
        model_pose = local_pose(model)
        for link in model.findall("link"):
            link_pose = compose(model_pose, local_pose(link))
            visuals = link.findall("visual")
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                if geometry is None or len(geometry) != 1:
                    raise ValueError("Each SDF collision must contain one primitive")
                primitive = geometry[0]
                shape = primitive.tag
                if shape == "box":
                    dimensions = numbers(primitive.findtext("size", ""), 3)
                elif shape == "cylinder":
                    dimensions = (float(primitive.findtext("length")), float(primitive.findtext("radius")))
                elif shape == "sphere":
                    dimensions = (float(primitive.findtext("radius")),)
                elif shape == "plane":
                    dimensions = numbers(primitive.findtext("normal", "0 0 1"), 3) + (0.0,)
                else:
                    raise ValueError(f"Unsupported SDF collision geometry: {shape}")
                if shape != "plane" and not all(math.isfinite(value) and value > 0 for value in dimensions):
                    raise ValueError("Primitive dimensions must be positive and finite")
                visual = next((v for v in visuals if v.get("name") == collision.get("name")),
                              visuals[0] if visuals else None)
                color = numbers(visual.findtext("material/diffuse", "0.5 0.5 0.5 1"), 4) if visual is not None else (0.5, 0.5, 0.5, 1.0)
                objects.append(SceneObject(
                    id=f"sdf/{model.attrib['name']}/{link.attrib['name']}/{collision.attrib['name']}",
                    shape=shape, dimensions=dimensions,
                    pose=compose(link_pose, local_pose(collision)), color=color,
                ))
    return WorldScene(world.attrib["name"], mount, tuple(objects), frames)
