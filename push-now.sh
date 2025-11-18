#!/bin/bash
# Manual push script - use this to test authentication
# After first successful push, commits will auto-push via the hook

BRANCH=$(git rev-parse --abbrev-ref HEAD)
echo "Pushing to origin/$BRANCH..."
git push -u origin "$BRANCH"

