# ROS2 Build Tool - Modular Version 0.3.0-alpha

**Automated robot workspace generation with sophisticated runtime introspection, Jinja2-templated configuration, and self-healing error recovery.**

Transform robot development from a multi-week expert task into a streamlined, automated deployment process.

## Overview

ROS2 Build Tool automates the complete setup of robot workspaces including:
- **Hardware drivers** with automatic discovery and integration
- **Navigation stack** (Nav2) with adaptive parameter generation
- **SLAM** (SLAM Toolbox, Cartographer, RTABMap)
- **Lifecycle management** for fault-tolerant operation
- **Self-healing** watchdog for automatic recovery
- **TF validation** using proper tf2_ros APIs
- **QoS validation** for topic compatibility
- **Diagnostic aggregation** for health monitoring
- **Foxglove/RViz** visualization
- **ros2_control** integration with hardware interfaces

## Features

### ✅ Fully Implemented (v0.3.0-alpha)

- **Modular Architecture**: 6 focused packages with clear separation of concerns
- **Pydantic Data Models**: Comprehensive validation with helpful error messages
- **Jinja2 Template System**: Validated code generation with syntax checking
- **Complete Orchestration Pipeline**: End-to-end workspace creation (NEW)
- **Repository Management**: GitHub cloning with retry logic and vcs integration (NEW)
- **Build Management**: colcon build wrapper with progress tracking (NEW)
- **Dependency Management**: rosdep with retry logic for reliable installs (NEW)
- **Xacro Support**: Automatic processing with temporary file cleanup
- **Enhanced URDF Parsing**: Proper 3D transforms with rotation matrices
- **ros2_control Integration**: Full hardware interface support with plugin detection
- **Lifecycle Node Management**: Sequential startup with OnStateTransition events
- **Self-Healing Watchdog**: Dynamic topic introspection + lifecycle recovery
- **TF Validation**: tf2_ros Buffer API with proper validation
- **QoS Validation**: Publisher/subscriber compatibility checking
- **Dynamic Nav2 Parameters**: Adaptive calculations based on robot specs
- **Diagnostic Aggregator**: Hierarchical health monitoring
- **Hardware Interface Discovery**: Runtime package and executable discovery
- **Composable Nodes**: Container support for performance optimization (NEW)
- **Foxglove Integration**: Web-based visualization and control (NEW)
- **Interactive Wizard**: questionary-based configuration with validation
- **Safe Platform Operations**: Permission checks, confirmation prompts, rollback
- **AST-based setup.py Parsing**: Robust entry point detection (IMPROVED)

### ⚠️ Partially Implemented

- **Integration Tests**: Framework ready, comprehensive tests pending
- **Dry-run Mode**: Implemented for preview, needs more coverage
- **Wizard UX**: No back button or preview yet

### 📋 Known Limitations

- **Windows Support**: Primary development on Linux, Windows testing in progress
- **Recovery system**: Works for lifecycle nodes only; regular nodes require `respawn=True`
- **Bounds checking**: For Nav2 parameters partially implemented

## Quick Start

### Installation

```bash
git clone https://github.com/jlbucklew/ros2-build-tool.git
cd ROS-Build-Tool

# Install all packages
cd ros2_build_tool_core && pip install -e . && cd ..
cd ros2_build_tool_hardware && pip install -e . && cd ..
cd ros2_build_tool_generators && pip install -e . && cd ..
cd ros2_build_tool_validation && pip install -e . && cd ..
cd ros2_build_tool_watchdog && pip install -e . && cd ..
cd ros2_build_tool_cli && pip install -e ".[wizard]" && cd ..
```

See [INSTALLATION.md](INSTALLATION.md) for details.

### Generate Your Robot Workspace

```bash
./ros2_build_tool.py --wizard
```

Or with URDF:

```bash
./ros2_build_tool.py --wizard --urdf my_robot.xacro --output ~/my_robot_ws
```

### Build and Run

```bash
cd ~/my_robot_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
ros2 launch my_robot_bringup robot.launch.py
```

See [USAGE.md](USAGE.md) for detailed examples.

## Architecture

### Package Structure

```
ROS-Build-Tool/
├── ros2_build_tool_core/          # Core data models and utilities
│   ├── models.py                  # Pydantic models with validation
│   ├── platform.py                # Platform detection
│   ├── environment.py             # ROS2 environment management
│   └── executor.py                # Command execution with retry
│
├── ros2_build_tool_hardware/      # Hardware discovery and URDF parsing
│   ├── urdf_parser.py             # Enhanced URDF parsing with xacro
│   ├── hardware_registry.py      # Pre-configured hardware components
│   ├── package_discovery.py      # Runtime package/executable discovery
│   └── hardware_interface_discovery.py  # ros2_control plugin detection
│
├── ros2_build_tool_generators/    # Launch and config generation
│   ├── launch_generator.py       # Lifecycle-aware launch files
│   └── nav2_parameter_generator.py # Dynamic Nav2 parameters
│
├── ros2_build_tool_validation/    # Validation tools
│   ├── tf_validator.py            # TF tree validation (tf2_ros)
│   ├── qos_validator.py           # QoS compatibility checking
│   └── diagnostic_config_generator.py  # Diagnostic aggregator config
│
├── ros2_build_tool_watchdog/      # Self-healing monitoring
│   └── topic_watchdog.py          # Topic monitoring with recovery
│
├── ros2_build_tool_cli/           # CLI interface
│   └── wizard.py                  # Interactive configuration wizard
│
├── ros2_build_tool.py             # Main orchestrator script
└── tests/                         # Comprehensive test suite
```

### Key Improvements Over Original

| Feature | Original | Improved |
|---------|----------|----------|
| **Architecture** | 2000+ line monolith | 6 modular packages |
| **Data Models** | Basic dataclasses | Pydantic with validation |
| **ros2_control** | "removed (unused)" | Full implementation |
| **Lifecycle** | respawn=True only | OnStateTransition events |
| **Self-Healing** | Mentioned in goals | Watchdog node implemented |
| **TF Validation** | Shell commands | tf2_ros Buffer API |
| **Xacro Support** | None | Automatic processing |
| **URDF Parsing** | Local geometry | Transform-aware |
| **Nav2 Parameters** | Some hardcoded | Fully dynamic |
| **Hardware Discovery** | Commented out | Complete plugin system |
| **Tests** | None | pytest suite |

See [IMPROVEMENTS.md](IMPROVEMENTS.md) for detailed comparison.

## Documentation

- **[INSTALLATION.md](INSTALLATION.md)** - Installation instructions
- **[USAGE.md](USAGE.md)** - Usage guide with examples
- **[IMPROVEMENTS.md](IMPROVEMENTS.md)** - Detailed feature documentation
- **[API Documentation](docs/)** - API reference (coming soon)

## Examples

### Example 1: Differential Drive Robot with Lidar

```bash
./ros2_build_tool.py --wizard
```

Configuration:
- Robot: `my_robot`
- Hardware: RPLidar, wheel encoders
- SLAM: SLAM Toolbox
- Navigation: Yes
- Lifecycle: Yes
- Self-healing: Yes

Generates complete workspace with:
- Sensor drivers with lifecycle management
- SLAM Toolbox configuration
- Nav2 with adaptive parameters (based on robot dimensions from URDF)
- Self-healing watchdog monitoring critical topics
- Diagnostic aggregator with hierarchical health monitoring

### Example 2: Custom Robot from URDF

```bash
./ros2_build_tool.py --wizard --urdf turtlebot3_burger.xacro
```

The tool:
1. Processes xacro to URDF
2. Extracts: width=0.281m, length=0.306m, wheel_radius=0.033m, wheel_separation=0.160m
3. Generates Nav2 parameters:
   - `desired_linear_vel`: 0.176 m/s (0.8 × max_velocity from URDF)
   - `lookahead_dist`: 0.211m (1.2 seconds × velocity)
   - `robot_radius`: 0.153m (max(width, length) / 2)
   - `inflation_radius`: 0.275m (1.8 × robot_radius)
4. Creates workspace with optimized configuration

## Development

### Running Tests

```bash
# Install dev dependencies
pip install -r requirements.txt

# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_models.py -v

# Run with coverage
pytest tests/ --cov=ros2_build_tool_core --cov-report=html
```

### Code Quality

```bash
# Format code
black ros2_build_tool_*/

# Lint
flake8 ros2_build_tool_*/

# Type checking
mypy ros2_build_tool_*/
```

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## Roadmap

### Current Version (0.3.0-alpha) - IN PROGRESS
- ✅ Jinja2 template-based generation
- ✅ Dynamic topic introspection for watchdog
- ✅ Proper 3D transform calculations
- ✅ Safe platform install with rollback
- ✅ Temporary file cleanup
- ⚠️ Bounds checking (partial)
- ⚠️ Improved error messages (partial)
- ❌ Dry-run mode
- ❌ Wizard improvements
- ❌ Integration tests

### Next Release (0.3.0-stable)
- Complete bounds checking for all Nav2 parameters
- Full integration test suite with launch_testing
- Wizard back button and preview
- Dry-run mode
- Documentation improvements

### Future Versions

**v0.4.0**: Advanced Features
- Plugin system for extensibility
- SLAM parameter optimization
- Multi-robot support
- Controller manager integration

**v0.5.0**: Production Readiness
- Performance benchmarking
- Sphinx documentation
- CI/CD pipeline
- Docker containerization

## System Requirements

- **OS**: Ubuntu 22.04 (Jammy) or 24.04 (Noble)
- **ROS2**: Humble or Jazzy
- **Python**: 3.8+
- **Disk Space**: 2GB for workspace + dependencies

## License

Apache License 2.0 - See [LICENSE](LICENSE) file

## Credits

Built on top of:
- [ROS2](https://docs.ros.org/en/humble/) - Robot Operating System
- [Nav2](https://navigation.ros.org/) - Navigation framework
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) - SLAM implementation
- [Pydantic](https://pydantic-docs.helpmanual.io/) - Data validation
- [questionary](https://github.com/tmbo/questionary) - Interactive CLI

## Support

- **Issues**: [GitHub Issues](https://github.com/jlbucklew/ros2-build-tool/issues)
- **Discussions**: [GitHub Discussions](https://github.com/jlbucklew/ros2-build-tool/discussions)
- **Email**: support@example.com

## Citation

If you use this tool in your research, please cite:

```bibtex
@software{ros2_build_tool,
  title = ROS2 Build Tool: Automated Robot Workspace Generation,
  author = John Bucklew,
  year = 2025,
  url = https://github.com/jlbucklew/ros2-build-tool
}
```

---

**Status**: ⚠️ Alpha Release (v0.3.0-alpha) - Active Development

Major improvements to self-healing, code generation, and safety. Not yet production-ready.

**What Works Well:**
- Dynamic topic introspection and lifecycle recovery
- Template-based launch generation with validation
- Proper 3D transform calculations
- Safe system modifications with rollback

**What Needs Work:**
- Integration testing
- Complete bounds checking
- Wizard UX improvements

See [next-steps.md](next-steps.md) for detailed critique and improvement roadmap.