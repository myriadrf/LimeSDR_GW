#!/bin/bash
set -e

# Default configuration
CONFIG_FILE="litex_config.py"
DEPS_DIR="deps"
VENV_DIR=".venv"
INSTALL=false
CHECK=false
FREEZE=false
BYPASS_CONFIG=false
EDITABLE=true

# Help message
function show_help {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --install        Install LiteX environment (using $CONFIG_FILE if present)"
    echo "  --check          Check currently installed LiteX versions against $CONFIG_FILE"
    echo "  --freeze         Save currently installed LiteX versions to $CONFIG_FILE"
    echo "  --new            Bypass $CONFIG_FILE and install newest LiteX repos"
    echo "  --non-editable   Install LiteX repos in non-editable mode"
    echo "  --help           Show this help message"
    echo ""
}

# Parse arguments
if [[ $# -eq 0 ]]; then
    show_help
    exit 1
fi

while [[ $# -gt 0 ]]; do
    case $1 in
        --install)
            INSTALL=true
            shift
            ;;
        --check)
            CHECK=true
            shift
            ;;
        --freeze)
            FREEZE=true
            shift
            ;;
        --new)
            BYPASS_CONFIG=true
            INSTALL=true
            shift
            ;;
        --non-editable)
            EDITABLE=false
            shift
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Check if we are in the root of the repo (should have boards/ or gateware/)
if [ ! -d "boards" ] || [ ! -d "gateware" ]; then
    echo "Error: This script must be run from the root of the LimeSDR_GW repository."
    exit 1
fi

# 1. Create and activate a local virtual environment
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment in $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR"/bin/activate

# Ensure pip is available and up to date
if ! python3 -m pip --version > /dev/null 2>&1; then
    echo "pip not found in virtual environment, attempting to install it..."
    python3 -m ensurepip --upgrade
fi
python3 -m pip install --upgrade pip

MANAGER="tools/litex_manager.py"
chmod +x "$MANAGER"

MANAGER_ARGS="--config $CONFIG_FILE --deps-dir $DEPS_DIR"
if [ "$EDITABLE" = false ]; then
    MANAGER_ARGS="$MANAGER_ARGS --non-editable"
fi

# 2. Handle Freeze
if [ "$FREEZE" = true ]; then
    python3 "$MANAGER" $MANAGER_ARGS freeze
fi

# 3. Handle Install
if [ "$INSTALL" = true ]; then
    if [ "$BYPASS_CONFIG" = true ]; then
        python3 "$MANAGER" $MANAGER_ARGS --new install
    else
        python3 "$MANAGER" $MANAGER_ARGS install
    fi
fi

# 4. Handle Check
if [ "$CHECK" = true ]; then
    python3 "$MANAGER" $MANAGER_ARGS check
fi

echo "-------------------------------------------------------"
if [ "$INSTALL" = true ]; then
    echo "LiteX installation/update complete."
fi
if [ "$FREEZE" = true ]; then
    echo "LiteX configuration saved to $CONFIG_FILE."
fi
echo "Run 'source $VENV_DIR/bin/activate' to use the environment."
echo "-------------------------------------------------------"
