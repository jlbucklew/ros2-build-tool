# ROS2 Build Tool - Production Release

## Status: Under Development (Production Restart)

This is a complete restart of the ROS2 Build Tool project with production-grade practices.

### Development Principles

1. **Test-First Development**: No code without tests
2. **CI/CD from Day One**: Every commit tested automatically
3. **Cross-Platform**: Linux and Windows support verified
4. **Docker-First**: All development in containers
5. **100% Critical Path Coverage**: No untested production code

### Project Structure

```
ros2_build_tool/
├── .github/
│   └── workflows/         # CI/CD pipelines
├── docker/                # Docker configurations
├── docs/                  # Documentation
├── src/                   # Source code
│   └── ros2_build_tool/   # Main package
├── tests/                 # Test suite
│   ├── unit/             # Unit tests
│   ├── integration/      # Integration tests
│   └── e2e/             # End-to-end tests
├── scripts/              # Development scripts
└── requirements/         # Dependencies
```

### Quality Standards

- Minimum 80% test coverage (enforced)
- All tests must pass before merge
- Cross-platform CI validation
- Security scanning on every commit
- Performance benchmarks required

### Development Workflow

1. Write failing test
2. Implement minimum code
3. Refactor with confidence
4. Document changes
5. Pass CI/CD checks

## Installation

```bash
# Development setup
git clone https://github.com/your-org/ros2-build-tool.git
cd ros2-build-tool
./scripts/setup-dev.sh
```

## Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov

# Run specific category
pytest -m unit
pytest -m integration
```

## License

Apache 2.0