"""
Command-line interface for ROS2 Build Tool with dry-run support
"""

import argparse
import logging
import sys
from pathlib import Path
from typing import Optional, Dict, Any
from ros2_build_tool_core.models import RobotProfile
from .wizard import Wizard


class DryRunMode:
    """Context manager for dry-run mode that prevents file writes"""

    def __init__(self, enabled: bool = False):
        self.enabled = enabled
        self.actions: list = []
        self._original_open = None
        self._original_mkdir = None

    def __enter__(self):
        if self.enabled:
            logging.info("=" * 60)
            logging.info("DRY RUN MODE - No files will be created or modified")
            logging.info("=" * 60)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.enabled:
            self._print_summary()
        return False

    def log_action(self, action_type: str, details: Dict[str, Any]):
        """Log an action that would be performed"""
        self.actions.append({
            'type': action_type,
            'details': details
        })

    def _print_summary(self):
        """Print summary of what would have been done"""
        logging.info("\n" + "=" * 60)
        logging.info("DRY RUN SUMMARY")
        logging.info("=" * 60)

        if not self.actions:
            logging.info("No actions would be performed.")
            return

        # Group actions by type
        actions_by_type = {}
        for action in self.actions:
            action_type = action['type']
            if action_type not in actions_by_type:
                actions_by_type[action_type] = []
            actions_by_type[action_type].append(action['details'])

        # Print grouped actions
        for action_type, details_list in actions_by_type.items():
            logging.info(f"\n{action_type.upper()}:")
            for details in details_list:
                if 'path' in details:
                    logging.info(f"  - {details['path']}")
                    if 'content_preview' in details:
                        preview = details['content_preview'][:100]
                        logging.info(f"    Preview: {preview}...")
                elif 'command' in details:
                    logging.info(f"  - {details['command']}")
                else:
                    logging.info(f"  - {details}")

        logging.info("\n" + "=" * 60)
        logging.info(f"Total actions: {len(self.actions)}")
        logging.info("To perform these actions, run without --dry-run")
        logging.info("=" * 60)


def setup_logging(verbose: bool = False):
    """Setup logging configuration"""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(levelname)s: %(message)s'
    )


def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description='ROS2 Build Tool - Automated robot workspace generation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run interactive wizard
  ros2-build-tool --wizard

  # Run with URDF file
  ros2-build-tool --wizard --urdf my_robot.urdf --output ~/my_workspace

  # Dry run to preview changes
  ros2-build-tool --wizard --dry-run

  # Load existing profile
  ros2-build-tool --profile my_robot.yaml --output ~/my_workspace
        """
    )

    # Input options
    parser.add_argument(
        '--wizard',
        action='store_true',
        help='Run interactive configuration wizard'
    )

    parser.add_argument(
        '--profile',
        type=Path,
        help='Load robot profile from YAML file'
    )

    parser.add_argument(
        '--urdf',
        type=Path,
        help='Path to robot URDF file (.urdf or .xacro)'
    )

    # Output options
    parser.add_argument(
        '--output',
        '-o',
        type=Path,
        help='Output workspace path (default: ~/ros2_ws)'
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Preview what would be generated without creating files'
    )

    # Behavior options
    parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Enable verbose logging'
    )

    parser.add_argument(
        '--force',
        '-f',
        action='store_true',
        help='Overwrite existing workspace'
    )

    parser.add_argument(
        '--skip-build',
        action='store_true',
        help='Skip colcon build phase (only generate structure and configs)'
    )

    parser.add_argument(
        '--skip-clone',
        action='store_true',
        help='Skip repository cloning (use existing repos in workspace)'
    )

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.verbose)

    # Validate arguments
    if not args.wizard and not args.profile:
        parser.error("Must specify either --wizard or --profile")

    if args.wizard and args.profile:
        parser.error("Cannot use both --wizard and --profile")

    # Create dry-run context
    with DryRunMode(enabled=args.dry_run) as dry_run:
        try:
            if args.wizard:
                profile = run_wizard(args, dry_run)
            else:
                profile = load_profile(args.profile, dry_run)

            if profile:
                generate_workspace(profile, args, dry_run)

                if not args.dry_run:
                    logging.info("\n✓ Workspace generated successfully!")
                    logging.info(f"  Workspace: {args.output or profile.workspace_path}")
                    logging.info("\nNext steps:")
                    logging.info("  1. cd " + str(args.output or profile.workspace_path))
                    logging.info("  2. rosdep install --from-paths src -y --ignore-src")
                    logging.info("  3. colcon build")
                    logging.info("  4. source install/setup.bash")
                    logging.info("  5. ros2 launch " + profile.name + "_bringup robot.launch.py")

            return 0

        except KeyboardInterrupt:
            logging.info("\nOperation cancelled by user")
            return 130

        except Exception as e:
            logging.error(f"Error: {e}")
            if args.verbose:
                import traceback
                traceback.print_exc()
            return 1


def run_wizard(args: argparse.Namespace, dry_run: DryRunMode) -> Optional[RobotProfile]:
    """Run interactive wizard"""
    wizard = Wizard()

    if dry_run.enabled:
        logging.info("Running wizard in dry-run mode...")
        logging.info("Configuration will be collected but not saved.\n")

    profile = wizard.run()

    # Override with command-line args
    if args.output:
        profile.workspace_path = args.output

    if args.urdf:
        profile.urdf_path = args.urdf

    if dry_run.enabled:
        dry_run.log_action('profile_create', {
            'name': profile.name,
            'ros_distro': profile.ros_distro,
            'hardware': profile.hardware,
            'slam_type': profile.slam_type.value,
            'navigation': profile.navigation
        })

    return profile


def load_profile(profile_path: Path, dry_run: DryRunMode) -> Optional[RobotProfile]:
    """Load profile from YAML file"""
    if not profile_path.exists():
        raise FileNotFoundError(f"Profile not found: {profile_path}")

    logging.info(f"Loading profile from: {profile_path}")
    profile = RobotProfile.from_yaml(profile_path)

    if dry_run.enabled:
        dry_run.log_action('profile_load', {
            'path': str(profile_path),
            'name': profile.name
        })

    return profile


def generate_workspace(profile: RobotProfile, args: argparse.Namespace, dry_run: DryRunMode):
    """Generate workspace files"""
    workspace_path = args.output or profile.workspace_path or Path.home() / 'ros2_ws'

    if dry_run.enabled:
        logging.info(f"\nWould generate workspace at: {workspace_path}")
        logging.info(f"  Profile: {profile.name}")
        logging.info(f"  ROS Distro: {profile.ros_distro}")
        logging.info(f"  Hardware: {', '.join(profile.hardware)}")

        # Log what would be created
        dry_run.log_action('directory_create', {'path': str(workspace_path / 'src')})
        dry_run.log_action('directory_create', {'path': str(workspace_path / f'src/{profile.name}_bringup')})
        dry_run.log_action('directory_create', {'path': str(workspace_path / f'src/{profile.name}_bringup/launch')})
        dry_run.log_action('directory_create', {'path': str(workspace_path / f'src/{profile.name}_bringup/config')})
        dry_run.log_action('directory_create', {'path': str(workspace_path / f'src/{profile.name}_bringup/urdf')})

        dry_run.log_action('file_create', {
            'path': str(workspace_path / f'src/{profile.name}_bringup/launch/robot.launch.py'),
            'content_preview': f'#!/usr/bin/env python3\n# Main launch file for {profile.name}'
        })

        dry_run.log_action('file_create', {
            'path': str(workspace_path / f'src/{profile.name}_bringup/launch/sensors.launch.py'),
            'content_preview': f'#!/usr/bin/env python3\n# Sensor drivers for {profile.name}'
        })

        if profile.navigation:
            dry_run.log_action('file_create', {
                'path': str(workspace_path / f'src/{profile.name}_bringup/config/nav2_params.yaml'),
                'content_preview': '# Nav2 parameters generated from robot specifications'
            })

        logging.info("\nDry run completed. See summary above for details.")
    else:
        # Actually generate workspace using orchestrator
        logging.info(f"Generating workspace at: {workspace_path}")

        from ros2_build_tool_core.orchestrator import WorkspaceOrchestrator
        from ros2_build_tool_hardware.hardware_registry import HardwareRegistry
        from pathlib import Path as PathLib

        # Initialize hardware registry
        data_dir = PathLib.home() / '.ros2_build_tool'
        data_dir.mkdir(exist_ok=True)
        hardware_registry = HardwareRegistry(data_dir, logging.getLogger('hardware_registry'))

        # Create orchestrator
        orchestrator = WorkspaceOrchestrator(logging.getLogger('orchestrator'))

        # Set up progress callback
        def progress_callback(message: str, percentage: int):
            logging.info(f"[{percentage}%] {message}")

        orchestrator.set_progress_callback(progress_callback)

        # Determine build options
        skip_build = getattr(args, 'skip_build', False)
        skip_clone = getattr(args, 'skip_clone', False)

        # Create workspace
        result = orchestrator.create_workspace(
            profile=profile,
            workspace_path=workspace_path,
            hardware_registry=hardware_registry,
            skip_build=skip_build,
            skip_clone=skip_clone
        )

        # Report results
        if result.success:
            logging.info("\n✓ Workspace created successfully!")
            logging.info(f"  Location: {result.workspace_path}")
            logging.info(f"  Packages: {len(result.created_packages)}")
            if result.cloned_repos:
                logging.info(f"  Cloned repos: {len(result.cloned_repos)}")

            if result.warnings:
                logging.warning("\nWarnings:")
                for warning in result.warnings:
                    logging.warning(f"  - {warning}")

        else:
            logging.error("\n✗ Workspace creation failed!")
            for error in result.errors:
                logging.error(f"  - {error}")

            # Offer rollback
            if workspace_path.exists():
                response = input(f"\nRollback workspace at {workspace_path}? [y/N]: ")
                if response.lower() == 'y':
                    orchestrator.rollback(workspace_path)

            sys.exit(1)


if __name__ == '__main__':
    sys.exit(main())