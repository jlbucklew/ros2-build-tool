"""
Interactive wizard interface for robot configuration
"""

from pathlib import Path
from ros2_build_tool_core.models import RobotProfile, UseCase, SLAMType

try:
    from questionary import prompt, Choice
    WIZARD_AVAILABLE = True
except ImportError:
    WIZARD_AVAILABLE = False


class Wizard:
    """Interactive wizard interface with questionary or fallback to basic input"""

    def __init__(self):
        self.profile = None

    def run(self) -> RobotProfile:
        """Run interactive wizard"""

        if WIZARD_AVAILABLE:
            return self._run_interactive()
        else:
            return self._run_basic()

    def _run_interactive(self) -> RobotProfile:
        """Run with questionary for better UX"""
        print("\n=== ROS2 Robot Configuration Wizard ===\n")

        questions = [
            {
                'type': 'text',
                'name': 'name',
                'message': 'Robot name:',
                'default': 'my_robot'
            },
            {
                'type': 'select',
                'name': 'ros_distro',
                'message': 'ROS2 distribution:',
                'choices': ['humble', 'jazzy']
            },
            {
                'type': 'select',
                'name': 'use_case',
                'message': 'Primary use case:',
                'choices': [
                    Choice('Mapping', UseCase.MAPPING),
                    Choice('Navigation', UseCase.NAVIGATION),
                    Choice('Perception', UseCase.PERCEPTION),
                    Choice('Full Stack', UseCase.FULL_STACK)
                ]
            },
            {
                'type': 'checkbox',
                'name': 'hardware',
                'message': 'Select hardware components:',
                'choices': [
                    Choice('RPLidar A1/A2', 'lidar_rplidar'),
                    Choice('Ouster OS1/OS2', 'lidar_ouster'),
                    Choice('Velodyne VLP-16', 'lidar_velodyne'),
                    Choice('V4L2 Camera', 'camera_v4l2'),
                    Choice('Intel RealSense', 'camera_realsense'),
                    Choice('Xsens MTi IMU', 'imu_xsens'),
                    Choice('Bosch BNO055 IMU', 'imu_bno055'),
                    Choice('u-blox GPS', 'gps_ublox'),
                    Choice('Wheel Encoders', 'wheel_encoders')
                ]
            },
            {
                'type': 'select',
                'name': 'slam_type',
                'message': 'SLAM algorithm:',
                'choices': [
                    Choice('SLAM Toolbox (recommended)', SLAMType.SLAM_TOOLBOX),
                    Choice('Cartographer', SLAMType.CARTOGRAPHER),
                    Choice('None', SLAMType.NONE)
                ]
            },
            {
                'type': 'confirm',
                'name': 'navigation',
                'message': 'Enable Nav2 navigation stack?',
                'default': False
            },
            {
                'type': 'confirm',
                'name': 'lifecycle_management',
                'message': 'Enable lifecycle node management?',
                'default': True
            },
            {
                'type': 'confirm',
                'name': 'self_healing',
                'message': 'Enable self-healing watchdog?',
                'default': True
            },
            {
                'type': 'confirm',
                'name': 'composable_nodes',
                'message': 'Use composable nodes for performance?',
                'default': False
            },
            {
                'type': 'confirm',
                'name': 'foxglove',
                'message': 'Enable Foxglove web visualization?',
                'default': True
            },
            {
                'type': 'confirm',
                'name': 'rviz',
                'message': 'Enable RViz visualization?',
                'default': True
            },
            {
                'type': 'confirm',
                'name': 'diagnostics',
                'message': 'Enable diagnostic aggregator?',
                'default': True
            },
            {
                'type': 'text',
                'name': 'workspace_path',
                'message': 'Workspace path:',
                'default': '~/ros2_ws'
            },
            {
                'type': 'text',
                'name': 'urdf_path',
                'message': 'Path to URDF file (optional, .urdf or .xacro):',
                'default': ''
            },
            {
                'type': 'text',
                'name': 'github_urls',
                'message': 'GitHub URLs for custom hardware drivers (comma-separated, optional):',
                'default': ''
            }
        ]

        answers = prompt(questions)

        # Process answers
        answers['workspace_path'] = Path(answers['workspace_path']).expanduser()

        # Process URDF path
        if answers.get('urdf_path') and answers['urdf_path'].strip():
            urdf_path = Path(answers['urdf_path']).expanduser()
            if urdf_path.exists():
                answers['urdf_path'] = urdf_path
            else:
                print(f"Warning: URDF path does not exist: {urdf_path}")
                answers['urdf_path'] = None
        else:
            answers['urdf_path'] = None

        # Process GitHub URLs
        custom_repos = {}
        if answers.get('github_urls') and answers['github_urls'].strip():
            github_urls = [url.strip() for url in answers['github_urls'].split(',') if url.strip()]
            for i, url in enumerate(github_urls):
                repo_name = url.split('/')[-1].replace('.git', '')
                custom_repos[repo_name] = {'url': url}

        answers['custom_repos'] = custom_repos
        answers.pop('github_urls', None)

        # Create profile
        try:
            profile = RobotProfile(**answers)
            print("\n✓ Configuration validated successfully!")
            return profile
        except Exception as e:
            print(f"\n✗ Configuration error: {e}")
            print("Please run the wizard again.")
            raise

    def _run_basic(self) -> RobotProfile:
        """Fallback to basic input when questionary not available"""
        print("\n=== ROS2 Robot Configuration Wizard (Basic Mode) ===")
        print("Note: Install 'questionary' for better experience: pip install questionary\n")

        name = input("Robot name [my_robot]: ").strip() or 'my_robot'

        print("\nROS2 distributions:")
        print("1. humble")
        print("2. jazzy")
        choice = input("Select (1-2) [1]: ").strip() or '1'
        ros_distro = 'humble' if choice == '1' else 'jazzy'

        print("\nUse cases:")
        print("1. Mapping")
        print("2. Navigation")
        print("3. Perception")
        print("4. Full Stack")
        choice = input("Select (1-4) [2]: ").strip() or '2'
        use_case = {
            '1': UseCase.MAPPING,
            '2': UseCase.NAVIGATION,
            '3': UseCase.PERCEPTION,
            '4': UseCase.FULL_STACK
        }.get(choice, UseCase.NAVIGATION)

        print("\nHardware components (comma-separated):")
        print("Available: lidar_rplidar, lidar_ouster, camera_v4l2, imu_xsens, wheel_encoders")
        hardware_input = input("Enter components [lidar_rplidar,wheel_encoders]: ").strip()
        if hardware_input:
            hardware = [h.strip() for h in hardware_input.split(',') if h.strip()]
        else:
            hardware = ['lidar_rplidar', 'wheel_encoders']

        print("\nSLAM algorithm:")
        print("1. SLAM Toolbox (recommended)")
        print("2. Cartographer")
        print("3. None")
        choice = input("Select (1-3) [1]: ").strip() or '1'
        slam_type = {
            '1': SLAMType.SLAM_TOOLBOX,
            '2': SLAMType.CARTOGRAPHER,
            '3': SLAMType.NONE
        }.get(choice, SLAMType.SLAM_TOOLBOX)

        navigation = input("\nEnable Nav2 navigation? (y/n) [y]: ").strip().lower() or 'y'
        navigation = navigation == 'y'

        lifecycle = input("Enable lifecycle management? (y/n) [y]: ").strip().lower() or 'y'
        lifecycle_management = lifecycle == 'y'

        healing = input("Enable self-healing watchdog? (y/n) [y]: ").strip().lower() or 'y'
        self_healing = healing == 'y'

        foxglove = input("Enable Foxglove? (y/n) [y]: ").strip().lower() or 'y'
        foxglove = foxglove == 'y'

        workspace = input("\nWorkspace path [~/ros2_ws]: ").strip() or '~/ros2_ws'
        workspace_path = Path(workspace).expanduser()

        urdf = input("URDF file path (optional): ").strip()
        urdf_path = Path(urdf).expanduser() if urdf else None

        try:
            profile = RobotProfile(
                name=name,
                ros_distro=ros_distro,
                use_case=use_case,
                hardware=hardware,
                slam_type=slam_type,
                navigation=navigation,
                lifecycle_management=lifecycle_management,
                self_healing=self_healing,
                foxglove=foxglove,
                workspace_path=workspace_path,
                urdf_path=urdf_path
            )
            print("\n✓ Configuration created successfully!")
            return profile
        except Exception as e:
            print(f"\n✗ Configuration error: {e}")
            print("Please run the wizard again.")
            raise

    def save_profile(self, profile: RobotProfile, path: Path):
        """Save profile to YAML file"""
        profile.to_yaml(path)
        print(f"✓ Profile saved to {path}")

    def load_profile(self, path: Path) -> RobotProfile:
        """Load profile from YAML file"""
        profile = RobotProfile.from_yaml(path)
        print(f"✓ Profile loaded from {path}")
        return profile