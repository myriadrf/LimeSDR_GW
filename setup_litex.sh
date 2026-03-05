#!/bin/bash
set -e

# Default configuration
CONFIG_FILE="litex_config.py"
DEPS_DIR="deps"
VENV_DIR=".venv"
KEEP_LITEX_SETUP=false
LITEX_SETUP_PRESENT=false
INSTALL=false
FREEZE=false
BYPASS_CONFIG=false
FORCE_KEEP=false

# Help message
function show_help {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --install        Install LiteX environment (using $CONFIG_FILE if present)"
    echo "  --freeze         Save currently installed LiteX versions to $CONFIG_FILE"
    echo "  --new            Bypass $CONFIG_FILE and install newest LiteX repos"
    echo "  --keep           Do not remove litex_setup.py even if it was downloaded"
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
        --freeze)
            FREEZE=true
            shift
            ;;
        --new)
            BYPASS_CONFIG=true
            shift
            ;;
        --keep)
            FORCE_KEEP=true
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

# 2. Create a directory for dependencies
mkdir -p "$DEPS_DIR"
cd "$DEPS_DIR"

# 3. Check if litex_setup.py is present
if [ -f "litex_setup.py" ]; then
    LITEX_SETUP_PRESENT=true
else
    echo "Downloading litex_setup.py..."
    wget https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py
    chmod +x litex_setup.py
fi

# 4. Handle Freeze
if [ "$FREEZE" = true ]; then
    echo "Freezing LiteX versions to ../$CONFIG_FILE..."
    # litex_setup.py --freeze output contains status messages, we only want the git_repos dictionary.
    # We use sed to extract from 'git_repos = {' to '}'
    ./litex_setup.py --freeze | sed -n '/git_repos = {/,$p' > "../$CONFIG_FILE"
    echo "Done."
fi

# 5. Handle Install
if [ "$INSTALL" = true ]; then
    INSTALL_ARGS="init install"
    
    if [ "$BYPASS_CONFIG" = true ]; then
        echo "Installing newest LiteX repositories (bypassing config)..."
        ./litex_setup.py $INSTALL_ARGS
    else
        if [ -f "../$CONFIG_FILE" ]; then
            echo "Installing LiteX using config: ../$CONFIG_FILE"
            # We create a temporary script to load the config and run litex_setup
            cat <<EOF > litex_setup_with_config.py
import sys
import os

# Add current directory to path so we can import litex_setup
sys.path.append(os.getcwd())
import litex_setup

# Inject GitRepo class into builtins so it's available for the config file
import builtins
builtins.GitRepo = litex_setup.GitRepo

# Add parent directory to path so we can import the config
sys.path.append(os.path.join(os.getcwd(), ".."))
import ${CONFIG_FILE%.py} as config

# Override git_repos with the ones from config
litex_setup.git_repos = config.git_repos

if __name__ == "__main__":
    # Remove the script name from args so litex_setup.main() sees only its own args
    sys.argv[0] = "litex_setup.py"
    litex_setup.main()
EOF
            python3 litex_setup_with_config.py $INSTALL_ARGS
            rm litex_setup_with_config.py
        else
            echo "Error: $CONFIG_FILE not found."
            echo "Use --new to install newest repos, or --freeze to create a config from an existing installation."
            # Cleanup if we downloaded litex_setup.py and not forced to keep it
            if [ "$LITEX_SETUP_PRESENT" = false ] && [ "$FORCE_KEEP" = false ]; then
                rm litex_setup.py
            fi
            exit 1
        fi
    fi
fi

# 6. Cleanup litex_setup.py
# If it wasn't present before AND --keep was not passed, remove it
if [ "$LITEX_SETUP_PRESENT" = false ] && [ "$FORCE_KEEP" = false ]; then
    echo "Removing downloaded litex_setup.py..."
    rm litex_setup.py
else
    echo "Keeping litex_setup.py (already present or --keep used)."
fi

cd ..

echo "-------------------------------------------------------"
if [ "$INSTALL" = true ]; then
    echo "LiteX installation/update complete."
fi
if [ "$FREEZE" = true ]; then
    echo "LiteX configuration saved to $CONFIG_FILE."
fi
echo "Run 'source $VENV_DIR/bin/activate' to use the environment."
echo "-------------------------------------------------------"
