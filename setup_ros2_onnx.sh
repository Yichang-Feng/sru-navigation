#!/bin/bash

set -e

VENV_NAME="ros2_onnx"
VENV_PATH="${HOME}/${VENV_NAME}"

echo "========================================="
echo "ROS2 + ONNX Runtime Environment Setup"
echo "========================================="
echo ""

# Check if virtual environment already exists
if [ -d "$VENV_PATH" ]; then
    echo "Warning: Virtual environment '${VENV_NAME}' already exists at ${VENV_PATH}"
    read -p "Do you want to remove it and create a new one? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing virtual environment..."
        rm -rf "$VENV_PATH"
    else
        echo "Aborting setup."
        exit 0
    fi
fi

# Check Python version
echo "Checking Python version..."
PYTHON_CMD=$(command -v python3.12)
if [ -z "$PYTHON_CMD" ]; then
    echo "Error: python3.12 not found. Please install Python 3.12."
    echo "On Ubuntu/Debian: sudo apt install python3.12 python3.12-venv"
    exit 1
fi

PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | awk '{print $2}')
echo "Using Python version: $PYTHON_VERSION"

# Create virtual environment
echo ""
echo "Creating virtual environment '${VENV_NAME}'..."
$PYTHON_CMD -m venv "$VENV_PATH"

# Activate virtual environment
echo "Activating virtual environment..."
source "${VENV_PATH}/bin/activate"

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip setuptools wheel

# Detect GPU availability
echo ""
echo "========================================="
echo "Detecting GPU availability..."
echo "========================================="

HAS_GPU=false

# Jetson 上不一定有 nvidia-smi，用 tegra-release / tegrastats 兜底
if [ -f /etc/nv_tegra_release ] || command -v tegrastats >/dev/null 2>&1; then
    echo "Jetson detected (CUDA is available on the device)."
    HAS_GPU=true
elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
    echo "NVIDIA GPU detected!"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
    HAS_GPU=true
else
    echo "No NVIDIA GPU detected. Will install CPU-only version of ONNX Runtime."
fi

# Jetson aarch64 通常需要 NVIDIA 的 wheel 索引；可用 ONNX_PIP_INDEX_URL 覆盖
ONNX_PIP_INDEX_URL="${ONNX_PIP_INDEX_URL:-}"
if [ -z "$ONNX_PIP_INDEX_URL" ] && [ "$(uname -m)" = "aarch64" ]; then
    ONNX_PIP_INDEX_URL="https://pypi.jetson-ai-lab.dev/jp6/cu126"
fi
ONNX_INDEX_ARGS=""
if [ -n "$ONNX_PIP_INDEX_URL" ]; then
    ONNX_INDEX_ARGS="--extra-index-url $ONNX_PIP_INDEX_URL"
fi

# Install ONNX Runtime
echo ""
echo "========================================="
echo "Installing ONNX Runtime..."
echo "========================================="

if [ "$HAS_GPU" = true ]; then
    echo "Installing ONNX Runtime with GPU support..."
    # 两个包装到同一目录，先都卸掉避免 CPU 版覆盖 GPU 版
    pip uninstall -y onnxruntime onnxruntime-gpu >/dev/null 2>&1 || true
    # shellcheck disable=SC2086
    pip install onnxruntime-gpu $ONNX_INDEX_ARGS

    # Verify GPU installation
    echo ""
    echo "Verifying ONNX Runtime GPU installation..."
    python -c "import onnxruntime as ort; providers = ort.get_available_providers(); print(f'ONNX Runtime version: {ort.__version__}'); print(f'Available providers: {providers}')"

    if python -c "import onnxruntime as ort; p=ort.get_available_providers(); exit(0 if ('CUDAExecutionProvider' in p or 'TensorrtExecutionProvider' in p) else 1)"; then
        echo ""
        echo "SUCCESS: ONNX Runtime with GPU support installed successfully!"
    else
        echo ""
        echo "WARNING: ONNX Runtime installed but CUDA/TensorRT provider not available."
        echo "Check that CUDA/cuDNN match the wheel (JetPack 6 / CUDA 12.6 -> jp6/cu126)."
        echo "You can override the index with ONNX_PIP_INDEX_URL=..."
    fi
else
    echo "Installing CPU-only version of ONNX Runtime..."
    pip install onnxruntime

    echo ""
    echo "Verifying ONNX Runtime CPU installation..."
    python -c "import onnxruntime as ort; print(f'ONNX Runtime version: {ort.__version__}'); print(f'Available providers: {ort.get_available_providers()}')"
fi

# Install other Python dependencies
echo ""
echo "========================================="
echo "Installing Python dependencies..."
echo "========================================="

pip install numpy scipy opencv-python pyyaml

# Install ROS2 Python dependencies (if not already available from system)
echo ""
echo "Installing additional dependencies..."
pip install setuptools

echo ""
echo "========================================="
echo "Setup Complete!"
echo "========================================="
echo ""
echo "Virtual environment '${VENV_NAME}' has been created at: ${VENV_PATH}"
echo ""
echo "To activate the environment, run:"
echo "    source ${VENV_PATH}/bin/activate"
echo ""
echo "To deactivate the environment, run:"
echo "    deactivate"
echo ""

if [ "$HAS_GPU" = true ]; then
    echo "ONNX Runtime with GPU support has been installed."
    echo "Your GPU will be used for neural network inference."
else
    echo "ONNX Runtime CPU-only version has been installed."
    echo "Neural network inference will run on CPU."
fi

echo ""
echo "Next steps:"
echo "1. Build the ROS2 package (do NOT activate venv for building):"
echo "   cd ~/sru_ws"
echo "   colcon build --packages-select rl_nav_controller --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "   Note: Do NOT use --symlink-install flag for the venv to work properly"
echo ""
echo "2. To run the navigation controller:"
echo "   source ${VENV_PATH}/bin/activate"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source ~/sru_ws/install/setup.bash"
echo "   ros2 launch rl_nav_controller rl_nav_controller.launch.py"
echo ""
