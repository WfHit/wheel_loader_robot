#!/bin/bash

################################################################################
# Localization Test Runner Script
################################################################################
# This script sets up the conda environment and runs the prior_coplanar_demo.py
# test to demonstrate how prior information helps with coplanar UWB anchors.
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}Localization Test Runner${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# Check if miniforge is installed
MINIFORGE_PATH="$HOME/miniforge3"
if [ ! -d "$MINIFORGE_PATH" ]; then
    echo -e "${RED}Error: Miniforge not found at $MINIFORGE_PATH${NC}"
    echo -e "${YELLOW}Please install miniforge first or update MINIFORGE_PATH in this script${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Found miniforge at: $MINIFORGE_PATH"

# Source conda
echo -e "${BLUE}Loading conda...${NC}"
source "$MINIFORGE_PATH/etc/profile.d/conda.sh"

# Check if environment exists
ENV_NAME="localization_test"
if ! conda env list | grep -q "^${ENV_NAME} "; then
    echo -e "${YELLOW}Environment '${ENV_NAME}' not found. Creating it...${NC}"
    conda create -n "$ENV_NAME" python=3.9 numpy matplotlib scipy -y
    echo -e "${GREEN}✓${NC} Environment created"
else
    echo -e "${GREEN}✓${NC} Environment '${ENV_NAME}' exists"
fi

# Activate environment
echo -e "${BLUE}Activating conda environment: ${ENV_NAME}${NC}"
conda activate "$ENV_NAME"

# Check if we're in the right directory
cd "$SCRIPT_DIR"
echo -e "${GREEN}✓${NC} Working directory: $SCRIPT_DIR"

# Check if test script exists
TEST_SCRIPT="prior_coplanar_demo.py"
if [ ! -f "$TEST_SCRIPT" ]; then
    echo -e "${RED}Error: Test script '$TEST_SCRIPT' not found in $SCRIPT_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Found test script: $TEST_SCRIPT"
echo ""

# Run the test
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}Running Test...${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

python "$TEST_SCRIPT"

EXIT_CODE=$?

echo ""
echo -e "${BLUE}================================${NC}"
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✓ Test completed successfully!${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    echo -e "${YELLOW}Output files generated:${NC}"
    echo -e "  • prior_coplanar_demonstration.png"
    echo ""
    echo -e "${YELLOW}To view the results:${NC}"
    echo -e "  cd $SCRIPT_DIR"
    echo -e "  xdg-open prior_coplanar_demonstration.png"
else
    echo -e "${RED}✗ Test failed with exit code: $EXIT_CODE${NC}"
    echo -e "${BLUE}================================${NC}"
fi

exit $EXIT_CODE
