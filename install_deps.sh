#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Detecting package manager and installing dependencies..."

if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y build-essential pkg-config libopencv-dev libv4l-dev cmake
  # optional: apriltag
  sudo apt-get install -y libapriltag-dev || true
  echo "Installed via apt-get"
elif command -v dnf >/dev/null 2>&1; then
  sudo dnf install -y gcc-c++ make pkgconfig opencv opencv-devel v4l-utils cmake || true
  sudo dnf install -y apriltag-devel || true
  echo "Installed via dnf"
elif command -v pacman >/dev/null 2>&1; then
  sudo pacman -Sy --noconfirm base-devel pkgconf opencv v4l-utils cmake || true
  sudo pacman -Sy --noconfirm apriltag || true
  echo "Installed via pacman"
elif command -v zypper >/dev/null 2>&1; then
  sudo zypper install -y gcc-c++ make pkg-config opencv-devel libv4l-devel cmake || true
  sudo zypper install -y apriltag-devel || true
  echo "Installed via zypper"
else
  echo "Unsupported package manager. Please install these packages manually:"
  echo "  - C++ build tools (g++/gcc, make)"
  echo "  - pkg-config"
  echo "  - OpenCV development packages (libopencv-dev / opencv-devel)"
  echo "  - libv4l (optional)"
  echo "  - cmake (optional)"
  exit 1
fi

echo
echo "Checking pkg-config for opencv4..."
if pkg-config --exists opencv4; then
  echo "pkg-config: opencv4 available"
else
  echo "Warning: pkg-config cannot find opencv4. You may need to install OpenCV dev packages or adjust PKG_CONFIG_PATH."
fi

echo
echo "Optional: If you want AprilTag support, ensure apriltag dev library is installed and pkg-config can find 'apriltag'"

echo
echo "Done. To build the project run from the project root:"
echo "  chmod +x ./build.sh && ./build.sh"
