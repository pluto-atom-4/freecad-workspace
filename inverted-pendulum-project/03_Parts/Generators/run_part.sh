#!/bin/bash
# Run FreeCAD part generation script with AppImage Python

APPIMAGE="$HOME/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage"
APPIMAGE_ROOT="$HOME/tmp/squashfs-root"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Create home tmp if needed
mkdir -p "$HOME/tmp"

# Extract AppImage if needed
if [ ! -d "$APPIMAGE_ROOT" ]; then
    echo "Extracting FreeCAD AppImage to $HOME/tmp..."
    cd "$HOME/tmp" && "$APPIMAGE" --appimage-extract > /dev/null 2>&1
fi

# Run script with AppImage Python
export PYTHONPATH="$APPIMAGE_ROOT/usr/lib/python3.11/site-packages:$APPIMAGE_ROOT/usr/lib:$PYTHONPATH"
export LD_LIBRARY_PATH="$APPIMAGE_ROOT/usr/lib:$LD_LIBRARY_PATH"

"$APPIMAGE_ROOT/usr/bin/python" "$SCRIPT_DIR/simple_part.py"
