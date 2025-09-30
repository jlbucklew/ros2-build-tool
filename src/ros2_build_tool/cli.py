"""Command-line interface for ROS2 Build Tool.

This module provides the CLI commands for the ROS2 Build Tool package.
"""

import sys
import logging
from pathlib import Path
from typing import Optional

import click
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich import print as rprint

from ros2_build_tool import (
    __version__,
    RobotSpec,
    URDFAnalyzer,
    RobotType
)

# Initialize console for rich output
console = Console()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@click.group()
@click.version_option(version=__version__)
@click.option('-v', '--verbose', is_flag=True, help='Enable verbose logging')
def main(verbose: bool) -> None:
    """ROS2 Build Tool - Production-grade automation for ROS2 workspaces.

    Generate complete ROS2 workspaces with Nav2 configuration, SLAM setup,
    and hardware integration from robot specifications.
    """
    if verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.debug("Verbose logging enabled")


@main.command()
@click.argument('spec_file', type=click.Path(exists=True, path_type=Path))
def validate(spec_file: Path) -> None:
    """Validate a robot specification file.

    \b
    SPEC_FILE: Path to robot specification YAML file

    Examples:
        rbt validate robot.yaml
        ros2-build-tool validate config/my_robot.yaml
    """
    try:
        console.print(f"[bold blue]Validating:[/bold blue] {spec_file}")

        # Load and validate spec
        spec = RobotSpec.from_yaml(spec_file)

        # Display validation results
        console.print("[bold green]✓[/bold green] Robot specification is valid!")

        # Show summary
        table = Table(title="Robot Specification Summary")
        table.add_column("Property", style="cyan", no_wrap=True)
        table.add_column("Value", style="magenta")

        table.add_row("Name", spec.name)
        table.add_row("Type", spec.type.value)
        table.add_row("Length", f"{spec.dimensions.length:.2f} m")
        table.add_row("Width", f"{spec.dimensions.width:.2f} m")
        table.add_row("Height", f"{spec.dimensions.height:.2f} m")
        table.add_row("Sensors", str(len(spec.sensors)))

        if spec.max_velocity:
            table.add_row("Max Linear Velocity", f"{spec.max_velocity.linear:.2f} m/s")
            table.add_row("Max Angular Velocity", f"{spec.max_velocity.angular:.2f} rad/s")

        console.print(table)

        sys.exit(0)

    except FileNotFoundError as e:
        console.print(f"[bold red]✗ Error:[/bold red] {e}")
        sys.exit(1)
    except ValueError as e:
        console.print(f"[bold red]✗ Validation Error:[/bold red] {e}")
        sys.exit(1)
    except Exception as e:
        console.print(f"[bold red]✗ Unexpected Error:[/bold red] {e}")
        logger.exception("Unexpected error during validation")
        sys.exit(1)


@main.command()
@click.argument('urdf_file', type=click.Path(exists=True, path_type=Path))
@click.option('--xacro', is_flag=True, help='Process file as xacro instead of URDF')
def analyze_urdf(urdf_file: Path, xacro: bool) -> None:
    """Analyze a URDF or xacro robot model file.

    \b
    URDF_FILE: Path to URDF or xacro file

    Examples:
        rbt analyze-urdf robot.urdf
        ros2-build-tool analyze-urdf robot.urdf.xacro --xacro
    """
    try:
        console.print(f"[bold blue]Analyzing:[/bold blue] {urdf_file}")

        # Parse URDF
        if xacro:
            analyzer = URDFAnalyzer.from_xacro(urdf_file)
        else:
            analyzer = URDFAnalyzer.from_file(urdf_file)

        console.print(f"[bold green]✓[/bold green] Successfully parsed robot model: {analyzer.robot_name}")

        # Display analysis results
        table = Table(title="URDF Analysis Results")
        table.add_column("Property", style="cyan", no_wrap=True)
        table.add_column("Value", style="magenta")

        table.add_row("Robot Name", analyzer.robot_name)
        table.add_row("Links", str(len(analyzer.links)))
        table.add_row("Joints", str(len(analyzer.joints)))

        frames = analyzer.get_frames()
        sensor_frames = analyzer.get_sensor_frames()
        table.add_row("Frames", str(len(frames)))
        table.add_row("Sensor Frames", str(len(sensor_frames)))

        # Validate TF tree
        validation = analyzer.validate_tf_tree()
        if validation.is_valid:
            table.add_row("TF Tree", "[green]Valid ✓[/green]")
        else:
            table.add_row("TF Tree", "[red]Invalid ✗[/red]")

        if validation.root_frame:
            table.add_row("Root Frame", validation.root_frame)

        console.print(table)

        # Show sensor frames
        if sensor_frames:
            console.print("\n[bold]Detected Sensor Frames:[/bold]")
            for frame in sensor_frames:
                parent_info = f" (parent: {frame.parent})" if frame.parent else ""
                console.print(f"  • {frame.name}{parent_info}")

        # Show validation issues
        if not validation.is_valid:
            console.print("\n[bold red]Validation Issues:[/bold red]")
            if not validation.has_single_root:
                console.print("  • TF tree does not have a single root frame")
            if validation.has_cycles:
                console.print("  • TF tree contains cycles")
            if validation.disconnected_frames:
                console.print(f"  • Disconnected frames: {', '.join(validation.disconnected_frames)}")

        sys.exit(0 if validation.is_valid else 1)

    except FileNotFoundError as e:
        console.print(f"[bold red]✗ Error:[/bold red] {e}")
        sys.exit(1)
    except Exception as e:
        console.print(f"[bold red]✗ Error:[/bold red] {e}")
        logger.exception("Error analyzing URDF")
        sys.exit(1)


@main.command()
@click.argument('output_file', type=click.Path(path_type=Path))
@click.option('--name', required=True, help='Robot name')
@click.option('--type', 'robot_type', required=True,
              type=click.Choice(['differential_drive', 'ackermann', 'omnidirectional', 'legged']),
              help='Robot type')
@click.option('--length', type=float, required=True, help='Robot length in meters')
@click.option('--width', type=float, required=True, help='Robot width in meters')
@click.option('--height', type=float, required=True, help='Robot height in meters')
def create_spec(output_file: Path, name: str, robot_type: str,
                length: float, width: float, height: float) -> None:
    """Create a new robot specification file.

    \b
    OUTPUT_FILE: Path where the specification file will be created

    Examples:
        rbt create-spec robot.yaml --name my_robot --type differential_drive --length 0.5 --width 0.3 --height 0.2
    """
    try:
        console.print(f"[bold blue]Creating robot specification:[/bold blue] {output_file}")

        # Create spec
        spec = RobotSpec(
            name=name,
            type=RobotType(robot_type),
            dimensions={
                'length': length,
                'width': width,
                'height': height
            }
        )

        # Save to file
        spec.to_yaml(output_file)

        console.print(f"[bold green]✓[/bold green] Robot specification created successfully!")
        console.print(f"  Saved to: {output_file}")

        sys.exit(0)

    except Exception as e:
        console.print(f"[bold red]✗ Error:[/bold red] {e}")
        logger.exception("Error creating specification")
        sys.exit(1)


@main.command()
def info() -> None:
    """Display information about ROS2 Build Tool.

    Shows version, features, and usage information.
    """
    panel = Panel.fit(
        f"""[bold cyan]ROS2 Build Tool[/bold cyan] v{__version__}

[bold]Features:[/bold]
  • Robot specification validation
  • URDF/xacro analysis
  • TF tree validation
  • Nav2 controller recommendations
  • Sensor frame detection

[bold]Commands:[/bold]
  • validate        Validate robot specification
  • analyze-urdf    Analyze URDF/xacro files
  • create-spec     Create new robot spec
  • info            Show this information

[bold]Usage:[/bold]
  ros2-build-tool --help
  rbt <command> --help

[dim]Homepage: https://github.com/your-org/ros2-build-tool[/dim]""",
        title="[bold]ROS2 Build Tool[/bold]",
        border_style="blue"
    )
    console.print(panel)
    sys.exit(0)


if __name__ == '__main__':
    main()