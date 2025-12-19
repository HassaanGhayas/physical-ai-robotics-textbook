#!/bin/bash
# Validation Service Runner Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
VENV_DIR=".venv"
DEFAULT_HOST="0.0.0.0"
DEFAULT_PORT="8000"

# Show usage
usage() {
    echo -e "${BLUE}Validation Service Runner${NC}"
    echo ""
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  api         Start the API server"
    echo "  validate    Run validation on a project"
    echo "  test        Run test suite"
    echo "  lint        Run linting checks"
    echo "  format      Format code"
    echo "  clean       Clean build artifacts"
    echo "  help        Show this help message"
    echo ""
    echo "API Options:"
    echo "  --host HOST     Host to bind to (default: ${DEFAULT_HOST})"
    echo "  --port PORT     Port to bind to (default: ${DEFAULT_PORT})"
    echo "  --reload        Enable auto-reload"
    echo "  --workers N     Number of workers (production)"
    echo ""
    echo "Validate Options:"
    echo "  --path PATH     Path to project (default: current directory)"
    echo "  --quality       Run quality checks only"
    echo "  --tests         Run tests only"
    echo "  --deployment    Run deployment checks only"
    echo "  --verbose       Enable verbose output"
    echo ""
    echo "Test Options:"
    echo "  --unit          Run unit tests only"
    echo "  --integration   Run integration tests only"
    echo "  --e2e           Run E2E tests only"
    echo "  --coverage      Generate coverage report"
    echo ""
    echo "Examples:"
    echo "  $0 api --reload"
    echo "  $0 validate --path /path/to/project"
    echo "  $0 test --unit --coverage"
}

# Activate virtual environment
activate_venv() {
    if [ -d "$VENV_DIR" ]; then
        source $VENV_DIR/bin/activate
    else
        echo -e "${YELLOW}Virtual environment not found. Run install.sh first.${NC}"
        exit 1
    fi
}

# Start API server
run_api() {
    echo -e "${GREEN}Starting Validation Service API...${NC}"

    local host=$DEFAULT_HOST
    local port=$DEFAULT_PORT
    local reload=""
    local workers=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --host)
                host="$2"
                shift 2
                ;;
            --port)
                port="$2"
                shift 2
                ;;
            --reload)
                reload="--reload"
                shift
                ;;
            --workers)
                workers="--workers $2"
                shift 2
                ;;
            *)
                shift
                ;;
        esac
    done

    echo -e "${BLUE}Host: ${host}${NC}"
    echo -e "${BLUE}Port: ${port}${NC}"

    if [ -n "$workers" ]; then
        # Production mode with Gunicorn
        gunicorn src.api.main:app \
            --bind ${host}:${port} \
            ${workers} \
            -k uvicorn.workers.UvicornWorker
    else
        # Development mode with Uvicorn
        uvicorn src.api.main:app \
            --host ${host} \
            --port ${port} \
            ${reload}
    fi
}

# Run validation
run_validate() {
    echo -e "${GREEN}Running Validation...${NC}"

    local path="."
    local opts=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --path)
                path="$2"
                shift 2
                ;;
            --quality)
                opts="$opts --no-tests --no-deployment"
                shift
                ;;
            --tests)
                opts="$opts --no-quality --no-deployment"
                shift
                ;;
            --deployment)
                opts="$opts --no-quality --no-tests"
                shift
                ;;
            --verbose)
                opts="$opts --verbose"
                shift
                ;;
            *)
                shift
                ;;
        esac
    done

    python -m src.cli run --path "$path" $opts
}

# Run tests
run_tests() {
    echo -e "${GREEN}Running Tests...${NC}"

    local test_path="tests/"
    local coverage=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --unit)
                test_path="tests/unit/ tests/core/ tests/quality/ tests/testing/ tests/deployment/"
                shift
                ;;
            --integration)
                test_path="tests/api/ tests/cli/"
                shift
                ;;
            --e2e)
                test_path="tests/e2e/"
                shift
                ;;
            --coverage)
                coverage="--cov=src --cov-report=html --cov-report=term"
                shift
                ;;
            *)
                shift
                ;;
        esac
    done

    pytest $test_path -v $coverage
}

# Run linting
run_lint() {
    echo -e "${GREEN}Running Linting...${NC}"

    echo -e "${BLUE}Running Ruff linter...${NC}"
    ruff check src/ tests/

    echo -e "${BLUE}Running Ruff formatter check...${NC}"
    ruff format --check src/ tests/

    echo -e "${BLUE}Running MyPy type check...${NC}"
    mypy src/ --ignore-missing-imports

    echo -e "${GREEN}All checks passed!${NC}"
}

# Format code
run_format() {
    echo -e "${GREEN}Formatting Code...${NC}"

    ruff check --fix src/ tests/
    ruff format src/ tests/

    echo -e "${GREEN}Code formatted!${NC}"
}

# Clean build artifacts
run_clean() {
    echo -e "${GREEN}Cleaning build artifacts...${NC}"

    rm -rf build/
    rm -rf dist/
    rm -rf *.egg-info/
    rm -rf .pytest_cache/
    rm -rf .mypy_cache/
    rm -rf .ruff_cache/
    rm -rf htmlcov/
    rm -rf .coverage
    rm -f coverage.xml
    find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
    find . -type f -name "*.pyc" -delete 2>/dev/null || true

    echo -e "${GREEN}Clean complete!${NC}"
}

# Main entry point
main() {
    if [ $# -eq 0 ]; then
        usage
        exit 0
    fi

    local command=$1
    shift

    # Activate venv for commands that need it
    case $command in
        api|validate|test|lint|format)
            activate_venv
            ;;
    esac

    case $command in
        api)
            run_api "$@"
            ;;
        validate)
            run_validate "$@"
            ;;
        test)
            run_tests "$@"
            ;;
        lint)
            run_lint
            ;;
        format)
            run_format
            ;;
        clean)
            run_clean
            ;;
        help|--help|-h)
            usage
            ;;
        *)
            echo -e "${RED}Unknown command: ${command}${NC}"
            usage
            exit 1
            ;;
    esac
}

main "$@"
