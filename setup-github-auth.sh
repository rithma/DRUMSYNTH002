#!/bin/bash
# Helper script to configure GitHub authentication

echo "=== GitHub Authentication Setup ==="
echo ""
echo "To get a Personal Access Token:"
echo "1. Go to: https://github.com/settings/tokens"
echo "2. Click 'Generate new token' → 'Generate new token (classic)'"
echo "3. Name it: 'DRUMSYNTH Auto Push'"
echo "4. Check 'repo' scope"
echo "5. Click 'Generate token' and COPY IT"
echo ""
echo "Then run this command:"
echo "  git push -u origin main"
echo ""
echo "When prompted:"
echo "  Username: rithma"
echo "  Password: [paste your token here]"
echo ""
echo "The token will be saved in your macOS keychain for future use."
echo ""

