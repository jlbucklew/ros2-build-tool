#!/bin/bash
# Development environment setup script
# Sets up complete development environment with all quality checks

set -e

echo "==================================="
echo "ROS2 Build Tool Development Setup"
echo "==================================="

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | grep -Po '(?<=Python )\d+\.\d+')
REQUIRED_VERSION="3.8"

if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
    echo "❌ Python $REQUIRED_VERSION or higher is required (found $PYTHON_VERSION)"
    exit 1
fi

echo "✅ Python version: $PYTHON_VERSION"

# Create virtual environment
echo "Creating virtual environment..."
python3 -m venv .venv
source .venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip setuptools wheel

# Install development dependencies
echo "Installing development dependencies..."
pip install -r requirements/dev.txt

# Install package in editable mode
echo "Installing package in editable mode..."
pip install -e .

# Set up pre-commit hooks
echo "Setting up pre-commit hooks..."
pre-commit install
pre-commit install --hook-type commit-msg

# Run initial quality checks
echo "Running initial quality checks..."

echo "1. Black formatting check..."
black --check src tests || {
    echo "Running Black to format code..."
    black src tests
}

echo "2. isort import check..."
isort --check-only src tests || {
    echo "Running isort to sort imports..."
    isort src tests
}

echo "3. Flake8 linting..."
flake8 src tests || echo "⚠️  Flake8 issues found (will be fixed during development)"

echo "4. MyPy type checking..."
mypy src || echo "⚠️  Type checking issues (expected before implementation)"

# Run tests (will fail initially - that's expected in TDD)
echo "Running tests (expected to fail before implementation)..."
pytest tests/unit -v || echo "⚠️  Tests failing (expected in TDD)"

echo ""
echo "==================================="
echo "✅ Development environment ready!"
echo "==================================="
echo ""
echo "Next steps:"
echo "1. Activate virtual environment: source .venv/bin/activate"
echo "2. Run tests: pytest"
echo "3. Run specific test: pytest tests/unit/test_robot_spec_parser.py -v"
echo "4. Run with coverage: pytest --cov"
echo "5. Run pre-commit checks: pre-commit run --all-files"
echo ""
echo "Remember: Write tests first, then implementation (TDD)!"