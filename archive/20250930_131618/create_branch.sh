#!/bin/bash
set -e

echo "=== Creating Feature Branch for Supervisor Remediation ==="

# Check current branch
CURRENT_BRANCH=$(git branch --show-current)
echo "Current branch: $CURRENT_BRANCH"

# Create and checkout the feature branch
echo "Creating feature branch: feature/supervisor-remediation"
git checkout -b feature/supervisor-remediation

# Verify we're on the correct branch
NEW_BRANCH=$(git branch --show-current)
echo "New branch: $NEW_BRANCH"

# Verify we're NOT on main
if [ "$NEW_BRANCH" = "main" ]; then
    echo "ERROR: Still on main branch!"
    exit 1
fi

# Show current HEAD
echo ""
echo "Current HEAD commit:"
git log --oneline -1

echo ""
echo "=== SUCCESS: Feature branch created ==="
echo "Branch: feature/supervisor-remediation"
echo "Status: Ready for remediation work"