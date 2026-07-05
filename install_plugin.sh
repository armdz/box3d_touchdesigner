#!/bin/sh
# Copies the built Box3D plugin bundles/dylib into TouchDesigner's global
# plugin folder on macOS.
# Build first with:
#   cmake -B build
#   cmake --build build --config Release

set -e

SRC="$(cd "$(dirname "$0")" && pwd)/plugin"
DEST="$HOME/Library/Application Support/Derivative/TouchDesigner099/Plugins"

if [ ! -d "$SRC/Box3DSolverCHOP.plugin" ]; then
    echo "[ERROR] No se encontro $SRC/Box3DSolverCHOP.plugin"
    echo "Compila primero:  cmake --build build --config Release"
    exit 1
fi

mkdir -p "$DEST"

# libBox3DCore.dylib must sit next to the .plugin bundles: they load it via
# an @loader_path-relative rpath so all three operators share one loaded
# instance (and therefore one world registry) instead of three isolated copies.
cp -R "$SRC"/*.plugin "$DEST"/
cp "$SRC"/libBox3DCore.dylib "$DEST"/

echo "[OK] Plugins copiados a: $DEST"
ls "$DEST"
