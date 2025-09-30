#!/usr/bin/env python3
"""
Run full test suite and save results as audit evidence.

This script executes the complete test suite with coverage reporting
and saves all output to a timestamped file for audit trail purposes.
"""

import subprocess
import sys
from datetime import datetime
from pathlib import Path


def main() -> int:
    """
    Execute pytest with full coverage and save results.
    
    Returns:
        Exit code from pytest (0 for success, non-zero for failure)
    """
    # Generate timestamp for results file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_file = Path(f"TEST_RESULTS_{timestamp}.txt")
    
    # Build pytest command
    cmd = [
        sys.executable,
        "-m",
        "pytest",
        "--cov=urdf_analyzer",
        "--cov-report=html",
        "--cov-report=term-missing",
        "-v",
        "--tb=short"
    ]
    
    print("=" * 80)
    print("URDF ANALYZER TEST SUITE - EVIDENCE COLLECTION")
    print("=" * 80)
    print(f"Timestamp: {datetime.now().isoformat()}")
    print(f"Command: {' '.join(cmd)}")
    print(f"Results file: {results_file}")
    print("=" * 80)
    print()
    
    # Run pytest and capture all output
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=False
        )
        
        # Combine stdout and stderr
        full_output = result.stdout
        if result.stderr:
            full_output += "\n\nSTDERR:\n" + result.stderr
        
        # Create header for results file
        header = [
            "URDF ANALYZER TEST SUITE RESULTS",
            "=" * 80,
            f"Executed: {datetime.now().isoformat()}",
            f"Command: {' '.join(cmd)}",
            f"Exit Code: {result.returncode}",
            "=" * 80,
            "",
        ]
        
        # Write results to file
        results_file.write_text(
            "\n".join(header) + "\n" + full_output,
            encoding="utf-8"
        )
        
        # Print output to console
        print(full_output)
        
        # Print summary
        print()
        print("=" * 80)
        print("EVIDENCE COLLECTION COMPLETE")
        print("=" * 80)
        print(f"✓ Results saved to: {results_file}")
        
        # Check for HTML coverage report
        htmlcov_index = Path("htmlcov/index.html")
        if htmlcov_index.exists():
            print(f"✓ HTML coverage report: {htmlcov_index}")
        else:
            print(f"⚠ HTML coverage report not found at: {htmlcov_index}")
        
        print()
        
        if result.returncode == 0:
            print("✓ ALL TESTS PASSED")
        else:
            print(f"✗ TESTS FAILED (exit code: {result.returncode})")
            print()
            print("CRITICAL: Tests must pass before proceeding with remediation.")
            print("Review the results file and fix failing tests.")
        
        print("=" * 80)
        
        return result.returncode
        
    except Exception as e:
        error_msg = f"ERROR: Failed to run tests: {e}"
        print(error_msg, file=sys.stderr)
        
        # Try to save error to file
        try:
            results_file.write_text(
                f"TEST EXECUTION FAILED\n"
                f"Timestamp: {datetime.now().isoformat()}\n"
                f"Error: {e}\n",
                encoding="utf-8"
            )
        except Exception:
            pass
        
        return 1


if __name__ == "__main__":
    sys.exit(main())