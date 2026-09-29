import math
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest


PACKAGE_SOURCE = Path(__file__).resolve().parents[1]
NUMERIC_ATTRIBUTES = {"xyz", "rpy", "rgba", "size", "radius", "length"}


def assert_same_xml(actual, expected):
    assert actual.tag == expected.tag
    assert actual.attrib.keys() == expected.attrib.keys()
    for name, expected_value in expected.attrib.items():
        actual_value = actual.attrib[name]
        if name in NUMERIC_ATTRIBUTES:
            assert [float(value) for value in actual_value.split()] == pytest.approx(
                [float(value) for value in expected_value.split()], abs=1e-12
            )
        else:
            assert actual_value == expected_value
    actual_children = sorted(actual, key=lambda child: (child.tag, child.get("name", "")))
    expected_children = sorted(expected, key=lambda child: (child.tag, child.get("name", "")))
    assert len(actual_children) == len(expected_children)
    for actual_child, expected_child in zip(actual_children, expected_children):
        assert_same_xml(actual_child, expected_child)


@pytest.mark.parametrize("prefix", ["", "kabot_"])
@pytest.mark.parametrize("description_file", [
    "kabot.urdf.xacro", "kabot_mock.urdf.xacro", "kabot_gazebo.urdf.xacro"
])
def test_urdf_xacro(prefix, description_file, tmp_path):
    if os.environ.get("KABOT_TEST_INSTALLED") == "1":
        from ament_index_python.packages import get_package_share_directory

        description_directory = Path(get_package_share_directory("kabot_robot"))
    else:
        description_directory = PACKAGE_SOURCE / "description"

    result = subprocess.run(
        ["xacro", str(description_directory / "urdf" / description_file), f"prefix:={prefix}",
         "controller_config:=/tmp/kabot_test_controllers.yaml"],
        check=True,
        capture_output=True,
        text=True,
    )
    urdf_file = tmp_path / "kabot.urdf"
    urdf_file.write_text(result.stdout)
    subprocess.run(["check_urdf", str(urdf_file)], check=True, capture_output=True, text=True)
    actual = ET.fromstring(result.stdout)
    if description_file == "kabot_gazebo.urdf.xacro":
        assert_physics(actual, prefix, urdf_file)
        plugin = actual.find("gazebo/plugin[@filename='gz_ros2_control-system']")
        assert plugin is not None
        assert plugin.findtext("parameters") == "/tmp/kabot_test_controllers.yaml"
        for gazebo in actual.findall("gazebo"):
            actual.remove(gazebo)
        for link in actual.findall("link"):
            for element in list(link):
                if element.tag in {"inertial", "collision"}:
                    link.remove(element)
    controls = actual.findall("ros2_control")
    if description_file != "kabot.urdf.xacro":
        assert len(controls) == 1
        control = controls[0]
        if description_file == "kabot_mock.urdf.xacro":
            assert control.attrib == {"name": prefix + "KabotMock", "type": "system"}
            assert control.findtext("hardware/plugin") == "mock_components/GenericSystem"
            assert control.findtext("hardware/param[@name='calculate_dynamics']") == "true"
        else:
            assert control.attrib == {"name": prefix + "KabotGazebo", "type": "system"}
            assert control.findtext("hardware/plugin") == "gz_ros2_control/GazeboSimSystem"
        joints = control.findall("joint")
        assert len(joints) == 2
        assert {joint.attrib["name"] for joint in joints} == {
            prefix + "left_wheel_joint", prefix + "right_wheel_joint"
        }
        for joint in joints:
            assert [item.attrib["name"] for item in joint.findall("command_interface")] == [
                "velocity"
            ]
            assert [item.attrib["name"] for item in joint.findall("state_interface")] == [
                "position", "velocity"
            ]
            for state in joint.findall("state_interface"):
                assert float(state.findtext("param[@name='initial_value']")) == 0.0
        actual.remove(control)
    else:
        assert not controls
    expected = ET.parse(PACKAGE_SOURCE.parent / "kabot.urdf").getroot()
    right_axis = expected.find("joint[@name='right_wheel_joint']/axis")
    assert right_axis is not None
    right_axis.set("xyz", "0 0 -1")

    for element in actual.iter():
        if element.tag in {"link", "joint", "parent", "child"}:
            attribute = "link" if element.tag in {"parent", "child"} else "name"
            assert element.attrib[attribute].startswith(prefix)
            element.set(attribute, element.attrib[attribute][len(prefix):])

    assert_same_xml(actual, expected)
    link_names = [link.attrib["name"] for link in actual.findall("link")]
    joint_names = [joint.attrib["name"] for joint in actual.findall("joint")]
    assert len(link_names) == len(set(link_names)) == 7
    assert len(joint_names) == len(set(joint_names)) == 6
    assert actual.find("ros2_control") is None

    for side, roll, axis_sign in [("left", -1.5, 1), ("right", 1.5, -1)]:
        joint = actual.find(f"joint[@name='{side}_wheel_joint']")
        assert joint is not None
        origin = joint.find("origin")
        axis = joint.find("axis")
        assert origin is not None and axis is not None
        assert [float(value) for value in origin.attrib["rpy"].split()] == [roll, 0, 0]
        assert [float(value) for value in axis.attrib["xyz"].split()] == [0, 0, axis_sign]
        assert -math.sin(roll) * axis_sign > 0
        assert abs(math.cos(roll) * axis_sign) > 0.01


def assert_physics(robot, prefix, urdf_file):
    masses = {"chassis": 0.150, "top": 0.020, "left_wheel": 0.010,
              "right_wheel": 0.010, "front_slider": 0.005, "back_slider": 0.005}
    for name, mass in masses.items():
        link = robot.find(f"link[@name='{prefix}{name}']")
        assert float(link.find("inertial/mass").get("value")) == mass
        assert_same_xml(link.find("collision/geometry"), link.find("visual/geometry"))
        inertia = link.find("inertial/inertia")
        diagonal = [float(inertia.get(key)) for key in ("ixx", "iyy", "izz")]
        assert all(value > 0 for value in diagonal)
        assert 2 * max(diagonal) <= sum(diagonal)
        assert all(float(inertia.get(key)) == 0 for key in ("ixy", "ixz", "iyz"))
    assert len(robot.findall("link/inertial")) == 6
    converted = subprocess.run(
        [shutil.which("gz") or "gz", "sdf", "-p", str(urdf_file)],
        check=True, capture_output=True, text=True
    )
    model = ET.fromstring(converted.stdout).find("model")
    assert model is not None
    assert sum(float(mass.text) for mass in model.findall("link/inertial/mass")) == pytest.approx(0.2)
    assert len(model.findall("link/collision")) == 6
    for collision in model.findall("link/collision"):
        name = collision.get("name")
        friction = 1.0 if "wheel" in name else 0.05 if "slider" in name else 0.5
        assert float(collision.findtext("surface/friction/ode/mu")) == friction
        assert float(collision.findtext("surface/friction/ode/mu2")) == friction
    assert {joint.get("name") for joint in model.findall("joint")} == {
        prefix + "left_wheel_joint", prefix + "right_wheel_joint"
    }
    for side, sign in [("left", 1), ("right", -1)]:
        joint = model.find(f"joint[@name='{prefix}{side}_wheel_joint']")
        assert joint.findtext("child") == prefix + side + "_wheel"
        assert [float(value) for value in joint.findtext("axis/xyz").split()] == [0, 0, sign]
