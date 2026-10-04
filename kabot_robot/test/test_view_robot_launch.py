import importlib.util
from pathlib import Path
import tomllib
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


PACKAGE_SOURCE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("installed", [False, True])
@pytest.mark.parametrize("fixed_frame", [None, "kabot_odom"])
def test_view_only_observes_controller(installed, fixed_frame):
    directory = Path(get_package_share_directory("kabot_robot")) if installed else PACKAGE_SOURCE / "description"
    spec = importlib.util.spec_from_file_location("view_robot", directory / "launch/view_robot.launch.py")
    view = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(view)
    context = LaunchContext()
    if fixed_frame:
        context.launch_configurations["fixed_frame"] = fixed_frame

    with patch.object(view, "Node", wraps=Node) as node_factory:
        entities = view.generate_launch_description().entities
    declarations = [entity for entity in entities if isinstance(entity, DeclareLaunchArgument)]
    assert {entity.name for entity in declarations} == {"rviz_config", "fixed_frame"}
    for declaration in declarations:
        declaration.execute(context)

    assert len(entities) == len(declarations) + 1
    assert sum(isinstance(entity, Node) for entity in entities) == 1
    node_factory.assert_called_once()
    options = node_factory.call_args.kwargs
    assert options["package"] == options["executable"] == "rviz2"
    assert "parameters" not in options
    arguments = [perform_substitutions(context, normalize_to_list_of_substitutions(argument))
                 for argument in options["arguments"]]
    assert arguments == ["-d", context.launch_configurations["rviz_config"], "-f", fixed_frame or "odom"]

    manager = yaml.safe_load(Path(arguments[1]).read_text())["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "odom"
    assert manager["Views"]["Current"]["Target Frame"] == "<Fixed Frame>"
    robot = next(display for display in manager["Displays"] if display["Class"] == "rviz_default_plugins/RobotModel")
    assert robot["Enabled"]
    assert robot["Description Source"] == "Topic"
    assert robot["Description Topic"]["Value"] == "/robot_description"
    assert robot["Description Topic"]["Durability Policy"] == "Transient Local"


def test_view_task_does_not_build_or_start_control():
    tasks = tomllib.loads((PACKAGE_SOURCE.parent / "pixi.toml").read_text())["tasks"]
    assert tasks["view"] == {"cmd": ["ros2", "launch", "kabot_robot", "view_robot.launch.py"]}