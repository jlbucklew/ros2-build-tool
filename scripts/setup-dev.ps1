# Development environment setup script for Windows
# Sets up complete development environment with all quality checks

$ErrorActionPreference = "Stop"

Write-Host "===================================" -ForegroundColor Cyan
Write-Host "ROS2 Build Tool Development Setup" -ForegroundColor Cyan
Write-Host "===================================" -ForegroundColor Cyan

# Check Python version
$pythonVersion = python --version 2>&1
if ($pythonVersion -match "Python (\d+\.\d+)") {
    $version = [version]$matches[1]
    $requiredVersion = [version]"3.8"

    if ($version -lt $requiredVersion) {
        Write-Host "❌ Python $requiredVersion or higher is required (found $version)" -ForegroundColor Red
        exit 1
    }
    Write-Host "✅ Python version: $version" -ForegroundColor Green
} else {
    Write-Host "❌ Could not determine Python version" -ForegroundColor Red
    exit 1
}

# Create virtual environment
Write-Host "Creating virtual environment..." -ForegroundColor Yellow
python -m venv .venv
& .\.venv\Scripts\Activate.ps1

# Upgrade pip
Write-Host "Upgrading pip..." -ForegroundColor Yellow
python -m pip install --upgrade pip setuptools wheel

# Install development dependencies
Write-Host "Installing development dependencies..." -ForegroundColor Yellow
pip install -r requirements\dev.txt

# Install package in editable mode
Write-Host "Installing package in editable mode..." -ForegroundColor Yellow
pip install -e .

# Set up pre-commit hooks
Write-Host "Setting up pre-commit hooks..." -ForegroundColor Yellow
pre-commit install
pre-commit install --hook-type commit-msg

# Run initial quality checks
Write-Host "Running initial quality checks..." -ForegroundColor Yellow

Write-Host "1. Black formatting check..." -ForegroundColor Cyan
try {
    black --check src tests
} catch {
    Write-Host "Running Black to format code..." -ForegroundColor Yellow
    black src tests
}

Write-Host "2. isort import check..." -ForegroundColor Cyan
try {
    isort --check-only src tests
} catch {
    Write-Host "Running isort to sort imports..." -ForegroundColor Yellow
    isort src tests
}

Write-Host "3. Flake8 linting..." -ForegroundColor Cyan
try {
    flake8 src tests
} catch {
    Write-Host "⚠️  Flake8 issues found (will be fixed during development)" -ForegroundColor Yellow
}

Write-Host "4. MyPy type checking..." -ForegroundColor Cyan
try {
    mypy src
} catch {
    Write-Host "⚠️  Type checking issues (expected before implementation)" -ForegroundColor Yellow
}

# Run tests (will fail initially - that's expected in TDD)
Write-Host "Running tests (expected to fail before implementation)..." -ForegroundColor Cyan
try {
    pytest tests\unit -v
} catch {
    Write-Host "⚠️  Tests failing (expected in TDD)" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "===================================" -ForegroundColor Green
Write-Host "✅ Development environment ready!" -ForegroundColor Green
Write-Host "===================================" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Cyan
Write-Host "1. Activate virtual environment: .\.venv\Scripts\Activate.ps1"
Write-Host "2. Run tests: pytest"
Write-Host "3. Run specific test: pytest tests\unit\test_robot_spec_parser.py -v"
Write-Host "4. Run with coverage: pytest --cov"
Write-Host "5. Run pre-commit checks: pre-commit run --all-files"
Write-Host ""
Write-Host "Remember: Write tests first, then implementation (TDD)!" -ForegroundColor Yellow