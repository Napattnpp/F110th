#!/usr/bin/env bash

# ==============================================================================
# Script to clean up and re-initialize the f1tenth_system submodule in F110th
# ==============================================================================

# Exit on any error
set -e

# Get the directory of the script to ensure paths are correct
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$SCRIPT_DIR"

cd "$PROJECT_ROOT"

echo "=== 1. Cleaning up existing f1tenth_system references ==="

# Deinitialize and remove the submodule from git cache if tracked
echo "Deinitializing submodule path 'src/f1tenth_system'..."
git submodule deinit -f src/f1tenth_system 2>/dev/null || true
git rm -f src/f1tenth_system 2>/dev/null || true
git rm -rf --cached src/f1tenth_system 2>/dev/null || true

# Remove leftover module metadata inside .git
echo "Removing Git module cache..."
rm -rf .git/modules/src/f1tenth_system

# Remove files from local disk
echo "Removing folder 'src/f1tenth_system' from disk..."
rm -rf src/f1tenth_system

# Clean up any residual .gitmodules staging entries
git reset HEAD src/f1tenth_system 2>/dev/null || true

echo "=== 2. Adding f1tenth_system as a Git submodule ==="
git submodule add -b humble-devel https://github.com/Napattnpp/f1tenth_system.git src/f1tenth_system

echo "=== 3. Initializing nested submodules recursively ==="
git submodule update --init --recursive

echo "=== Submodule initialization complete! ==="
echo "You can now verify changes with 'git status' and commit them."
