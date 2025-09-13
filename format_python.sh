#!/bin/bash

# Python Code Formatting Script for Drone Perimeter Surveillance Project
# ========================================================================
#
# This script runs all Python code formatting and linting tools in the correct order.
# Designed for ROS2 projects with mixed C++/Python codebase.
#
# Usage:
#   ./format_python.sh [--check] [--fix] [path]
#
# Options:
#   --check    Only check formatting, don't modify files
#   --fix      Automatically fix issues where possible
#   path       Specific path to format (default: entire project)
#
# Author: Drone Perimeter Surveillance Team
# License: Apache 2.0

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
CHECK_ONLY=false
AUTO_FIX=false
TARGET_PATH="."

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --check)
            CHECK_ONLY=true
            shift
            ;;
        --fix)
            AUTO_FIX=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--check] [--fix] [path]"
            echo "  --check    Only check formatting, don't modify files"
            echo "  --fix      Automatically fix issues where possible"
            echo "  path       Specific path to format (default: entire project)"
            exit 0
            ;;
        *)
            TARGET_PATH="$1"
            shift
            ;;
    esac
done

echo -e "${BLUE}🚁 Drone Perimeter Surveillance - Python Code Formatter${NC}"
echo -e "${BLUE}============================================================${NC}"
echo

# Check if we're in the project root
if [[ ! -f "pyproject.toml" ]]; then
    echo -e "${RED}❌ Error: pyproject.toml not found. Please run this script from the project root.${NC}"
    exit 1
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to get Python executable path
get_python_cmd() {
    # Check if we're in a virtual environment
    if [[ -f ".venv/bin/python" ]]; then
        echo ".venv/bin/python"
    elif [[ -f ".venv/Scripts/python.exe" ]]; then
        echo ".venv/Scripts/python.exe"
    else
        echo "python3"
    fi
}

# Function to run a formatting tool
run_formatter() {
    local tool_name="$1"
    local command="$2"
    local description="$3"

    echo -e "${YELLOW}📋 Running ${tool_name}: ${description}${NC}"

    # Check if command exists (handle both direct commands and python -m)
    local check_cmd=$(echo $command | cut -d' ' -f1)
    if [[ "$check_cmd" == *"python"* ]]; then
        # For python -m commands, check if python exists
        local python_cmd=$(get_python_cmd)
        if ! command_exists "$python_cmd"; then
            echo -e "${RED}❌ Python not found. Please ensure Python is installed.${NC}"
            return 1
        fi
        # Replace python placeholder with actual python command
        command=$(echo "$command" | sed "s|python|$python_cmd|g")
    elif ! command_exists "$check_cmd"; then
        echo -e "${RED}❌ ${tool_name} not found. Please install it first.${NC}"
        return 1
    fi

    if eval "$command"; then
        echo -e "${GREEN}✅ ${tool_name} completed successfully${NC}"
        return 0
    else
        echo -e "${RED}❌ ${tool_name} found issues${NC}"
        return 1
    fi
}

# Python files pattern (excluding build/install/log directories)
PYTHON_FILES_PATTERN="--include=\"\.pyi?$\" --extend-exclude=\"/(build|install|log|logs|\.git|\.venv|__pycache__)/\""

EXIT_CODE=0

echo -e "${BLUE}🎯 Target: ${TARGET_PATH}${NC}"
echo -e "${BLUE}🔧 Mode: $(if $CHECK_ONLY; then echo "Check Only"; elif $AUTO_FIX; then echo "Auto-fix"; else echo "Format"; fi)${NC}"
echo

# 1. Import sorting with isort
echo -e "${YELLOW}=== Step 1: Import Sorting ===${NC}"
if $CHECK_ONLY; then
    ISORT_CMD="python -m isort --check-only --diff --settings-path=pyproject.toml \"$TARGET_PATH\" --skip-glob=\"*/install/*\" --skip-glob=\"*/build/*\" --skip-glob=\"*/log/*\" --skip-glob=\"*/logs/*\""
else
    ISORT_CMD="python -m isort --settings-path=pyproject.toml \"$TARGET_PATH\" --skip-glob=\"*/install/*\" --skip-glob=\"*/build/*\" --skip-glob=\"*/log/*\" --skip-glob=\"*/logs/*\""
fi

if ! run_formatter "isort" "$ISORT_CMD" "Sort Python imports"; then
    EXIT_CODE=1
fi
echo

# 2. Code formatting with Black
echo -e "${YELLOW}=== Step 2: Code Formatting ===${NC}"
if $CHECK_ONLY; then
    BLACK_CMD="python -m black --check --diff --config=pyproject.toml \"$TARGET_PATH\""
else
    BLACK_CMD="python -m black --config=pyproject.toml \"$TARGET_PATH\""
fi

if ! run_formatter "black" "$BLACK_CMD" "Format Python code"; then
    EXIT_CODE=1
fi
echo

# 3. Linting with flake8
echo -e "${YELLOW}=== Step 3: Code Linting ===${NC}"
FLAKE8_CMD="python -m flake8 \"$TARGET_PATH\" --exclude=build,install,log,logs,.venv,__pycache__"

if ! run_formatter "flake8" "$FLAKE8_CMD" "Lint Python code"; then
    EXIT_CODE=1
fi
echo

# 4. Type checking with mypy (only if not in check mode or if auto-fix is disabled)
if ! $CHECK_ONLY; then
    echo -e "${YELLOW}=== Step 4: Type Checking ===${NC}"
    MYPY_CMD="python -m mypy --config-file=pyproject.toml \"$TARGET_PATH\""

    if ! run_formatter "mypy" "$MYPY_CMD" "Check Python types"; then
        EXIT_CODE=1
    fi
    echo
fi

# 5. Security scanning with bandit
echo -e "${YELLOW}=== Step 5: Security Scanning ===${NC}"
BANDIT_CMD="python -m bandit -r \"$TARGET_PATH\" --configfile=pyproject.toml --quiet --exclude=\"*/build/*,*/install/*,*/log/*,*/logs/*,*/.venv/*\""

if ! run_formatter "bandit" "$BANDIT_CMD" "Scan for security issues"; then
    EXIT_CODE=1
fi
echo

# 6. ROS2-specific checks (if available) - Make these warnings, not errors
echo -e "${YELLOW}=== Step 6: ROS2-Specific Checks ===${NC}"

# Check for ament_pep257 (docstring checking) - Don't fail on issues
if command_exists ament_pep257; then
    echo -e "${YELLOW}📋 Running ament_pep257: Check docstring compliance${NC}"
    if ament_pep257 "$TARGET_PATH" 2>/dev/null; then
        echo -e "${GREEN}✅ ament_pep257 completed successfully${NC}"
    else
        echo -e "${YELLOW}⚠️  ament_pep257 found docstring style issues (warnings only)${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  ament_pep257 not available (install ros-jazzy-ament-pep257)${NC}"
fi

# Check for ament_copyright - Don't fail on issues
if command_exists ament_copyright; then
    echo -e "${YELLOW}📋 Running ament_copyright: Check copyright headers${NC}"
    if ament_copyright "$TARGET_PATH" 2>/dev/null; then
        echo -e "${GREEN}✅ ament_copyright completed successfully${NC}"
    else
        echo -e "${YELLOW}⚠️  ament_copyright found missing headers (warnings only)${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  ament_copyright not available (install ros-jazzy-ament-copyright)${NC}"
fi
echo

# Final summary
echo -e "${BLUE}============================================================${NC}"
if [[ $EXIT_CODE -eq 0 ]]; then
    echo -e "${GREEN}🎉 All formatting checks passed!${NC}"
    if ! $CHECK_ONLY; then
        echo -e "${GREEN}✨ Code has been formatted and is ready for commit.${NC}"
    fi
else
    echo -e "${RED}❌ Some formatting issues were found.${NC}"
    if $CHECK_ONLY; then
        echo -e "${YELLOW}💡 Run './format_python.sh --fix' to automatically fix issues.${NC}"
    fi
fi

echo -e "${BLUE}============================================================${NC}"
exit $EXIT_CODE
