#!/bin/bash
# Validation Service Installation Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
PYTHON_VERSION="3.11"
VENV_DIR=".venv"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Validation Service Installation Script${NC}"
echo -e "${GREEN}========================================${NC}"

# Check Python version
check_python() {
    echo -e "\n${YELLOW}Checking Python version...${NC}"

    if command -v python3 &> /dev/null; then
        PYTHON_CMD="python3"
    elif command -v python &> /dev/null; then
        PYTHON_CMD="python"
    else
        echo -e "${RED}Error: Python not found${NC}"
        echo "Please install Python ${PYTHON_VERSION} or higher"
        exit 1
    fi

    VERSION=$($PYTHON_CMD -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
    echo -e "Found Python version: ${GREEN}${VERSION}${NC}"

    # Check if version is at least 3.10
    if [[ $(echo "$VERSION 3.10" | awk '{print ($1 >= $2)}') -eq 0 ]]; then
        echo -e "${RED}Error: Python 3.10 or higher is required${NC}"
        exit 1
    fi
}

# Create virtual environment
create_venv() {
    echo -e "\n${YELLOW}Creating virtual environment...${NC}"

    if [ -d "$VENV_DIR" ]; then
        echo -e "${YELLOW}Virtual environment already exists. Skipping creation.${NC}"
    else
        $PYTHON_CMD -m venv $VENV_DIR
        echo -e "${GREEN}Virtual environment created at ${VENV_DIR}${NC}"
    fi

    # Activate virtual environment
    source $VENV_DIR/bin/activate
    echo -e "${GREEN}Virtual environment activated${NC}"
}

# Install dependencies
install_dependencies() {
    echo -e "\n${YELLOW}Installing dependencies...${NC}"

    # Upgrade pip
    pip install --upgrade pip

    # Install main dependencies
    if [ -f "requirements.txt" ]; then
        pip install -r requirements.txt
        echo -e "${GREEN}Main dependencies installed${NC}"
    else
        echo -e "${RED}Warning: requirements.txt not found${NC}"
    fi

    # Install dev dependencies if in development mode
    if [ "$1" = "--dev" ] || [ "$1" = "-d" ]; then
        if [ -f "requirements-dev.txt" ]; then
            pip install -r requirements-dev.txt
            echo -e "${GREEN}Development dependencies installed${NC}"
        fi
    fi
}

# Install the package
install_package() {
    echo -e "\n${YELLOW}Installing validation service package...${NC}"

    if [ "$1" = "--dev" ] || [ "$1" = "-d" ]; then
        pip install -e ".[dev]"
    else
        pip install -e .
    fi

    echo -e "${GREEN}Package installed${NC}"
}

# Verify installation
verify_installation() {
    echo -e "\n${YELLOW}Verifying installation...${NC}"

    # Check if CLI is available
    if command -v validation-service &> /dev/null; then
        echo -e "${GREEN}CLI installed successfully${NC}"
        validation-service --version
    else
        echo -e "${YELLOW}CLI not in PATH. Try: ${NC}"
        echo -e "  source ${VENV_DIR}/bin/activate"
        echo -e "  validation-service --help"
    fi

    # Check imports
    $PYTHON_CMD -c "from src.core.pipeline import ValidationPipeline; print('Core imports OK')"
    $PYTHON_CMD -c "from src.api.main import create_app; print('API imports OK')"

    echo -e "${GREEN}Installation verified successfully${NC}"
}

# Create necessary directories
create_directories() {
    echo -e "\n${YELLOW}Creating necessary directories...${NC}"

    mkdir -p data/runs
    mkdir -p data/reports
    mkdir -p logs

    echo -e "${GREEN}Directories created${NC}"
}

# Generate configuration
generate_config() {
    echo -e "\n${YELLOW}Setting up configuration...${NC}"

    if [ ! -f ".env" ]; then
        if [ -f ".env.example" ]; then
            cp .env.example .env
            echo -e "${GREEN}Created .env from .env.example${NC}"
        else
            cat > .env << EOF
# Validation Service Configuration
VALIDATION_ENV=development
LOG_LEVEL=INFO

# Quality Validation
ENABLE_QUALITY_VALIDATION=true
MIN_COVERAGE_PERCENTAGE=80

# Test Execution
ENABLE_TEST_EXECUTION=true
PARALLEL_EXECUTION=true

# Deployment Validation
ENABLE_DEPLOYMENT_VALIDATION=true
EOF
            echo -e "${GREEN}Created default .env configuration${NC}"
        fi
    else
        echo -e "${YELLOW}.env file already exists. Skipping.${NC}"
    fi
}

# Print success message
print_success() {
    echo -e "\n${GREEN}========================================${NC}"
    echo -e "${GREEN}Installation Complete!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "\nTo get started:"
    echo -e "  ${YELLOW}source ${VENV_DIR}/bin/activate${NC}"
    echo -e "  ${YELLOW}validation-service --help${NC}"
    echo -e "\nTo run the API server:"
    echo -e "  ${YELLOW}uvicorn src.api.main:app --reload${NC}"
    echo -e "\nTo run tests:"
    echo -e "  ${YELLOW}pytest${NC}"
}

# Main installation flow
main() {
    check_python
    create_venv
    install_dependencies "$1"
    install_package "$1"
    create_directories
    generate_config
    verify_installation
    print_success
}

# Run main with arguments
main "$@"
