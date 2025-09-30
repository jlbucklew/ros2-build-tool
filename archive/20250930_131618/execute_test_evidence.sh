#!/bin/bash
# Execute test evidence collection script

echo "Starting test evidence collection..."
python run_tests_with_evidence.py

# Check exit code
EXIT_CODE=$?

echo ""
echo "Test execution complete with exit code: $EXIT_CODE"

# List generated files
echo ""
echo "Generated evidence files:"
ls -lh TEST_RESULTS_*.txt 2>/dev/null || echo "No TEST_RESULTS files found"

if [ -d "htmlcov" ]; then
    echo ""
    echo "HTML coverage report:"
    ls -lh htmlcov/index.html 2>/dev/null || echo "No htmlcov/index.html found"
fi

exit $EXIT_CODE