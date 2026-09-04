#!/bin/bash
# Stage 0 — Acquire & license-check TurtleBot3 Burger assets.
#
# Fetches the reference URDF + wheel/base meshes for the ROBOTIS TurtleBot3
# Burger from ROBOTIS-GIT/turtlebot3, pinned to a fixed commit SHA (not a
# moving branch ref), verifies the license at that SHA is Apache-2.0, and
# writes ../NOTICE with the exact provenance record.
#
# Assets land in reference/ which is gitignored — this script re-fetches
# them on demand; nothing here is committed to the repo (decision #3).
#
# Usage: ./00_fetch_turtlebot3_assets.sh [PINNED_SHA]
#   PINNED_SHA defaults to the SHA baked in below. Pass an override only if
#   you have deliberately decided to re-pin (and update the default below).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REFERENCE_DIR="$SCRIPT_DIR/reference"
NOTICE_FILE="$SCRIPT_DIR/NOTICE"

# Pinned at implementation time by resolving:
#   git ls-remote https://github.com/ROBOTIS-GIT/turtlebot3.git refs/heads/main
# Re-run that command and pass the new SHA as $1 to deliberately re-pin.
DEFAULT_SHA="fc817ce3073af1d6032397c64504134882af5e9a"
PINNED_SHA="${1:-$DEFAULT_SHA}"

REPO_URL="https://github.com/ROBOTIS-GIT/turtlebot3"
RAW_BASE="https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/${PINNED_SHA}"

# Files pulled from turtlebot3_description at the pinned SHA.
# (lds.stl intentionally skipped per plan — not needed for base/wheel articulation POC.)
declare -a FILES=(
  "turtlebot3_description/urdf/turtlebot3_burger.urdf"
  "turtlebot3_description/meshes/bases/burger_base.stl"
  "turtlebot3_description/meshes/wheels/left_tire.stl"
  "turtlebot3_description/meshes/wheels/right_tire.stl"
)

echo "=================================================================="
echo "Stage 0: Fetch TurtleBot3 Burger assets"
echo "  Repo:   $REPO_URL"
echo "  SHA:    $PINNED_SHA"
echo "=================================================================="

mkdir -p "$REFERENCE_DIR/meshes"

echo ""
echo "-- License check --"
LICENSE_URL="$RAW_BASE/LICENSE"
LICENSE_TMP="$(mktemp)"
if ! curl -fsSL "$LICENSE_URL" -o "$LICENSE_TMP"; then
  echo "FATAL: could not fetch LICENSE at $LICENSE_URL" >&2
  rm -f "$LICENSE_TMP"
  exit 1
fi

if ! grep -qi "Apache License" "$LICENSE_TMP" || ! grep -q "Version 2.0" "$LICENSE_TMP"; then
  echo "FATAL: LICENSE at pinned SHA does not look like Apache License 2.0 — aborting." >&2
  echo "First lines of fetched LICENSE:" >&2
  head -n 5 "$LICENSE_TMP" >&2
  rm -f "$LICENSE_TMP"
  exit 1
fi
echo "OK: LICENSE at $PINNED_SHA confirmed Apache License 2.0"
rm -f "$LICENSE_TMP"

echo ""
echo "-- Fetching files --"
FETCHED_FILES=()
for rel_path in "${FILES[@]}"; do
  filename="$(basename "$rel_path")"
  case "$rel_path" in
    *urdf/*) dest="$REFERENCE_DIR/$filename" ;;
    *meshes/*) dest="$REFERENCE_DIR/meshes/$filename" ;;
    *) dest="$REFERENCE_DIR/$filename" ;;
  esac
  url="$RAW_BASE/$rel_path"
  echo "  GET $url"
  if ! curl -fsSL "$url" -o "$dest"; then
    echo "FATAL: failed to fetch $url" >&2
    exit 1
  fi
  size=$(du -h "$dest" | cut -f1)
  echo "    -> $dest ($size)"
  FETCHED_FILES+=("$rel_path")
done

echo ""
echo "-- Writing NOTICE --"
RETRIEVAL_DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
{
  echo "TurtleBot3 asset attribution (Apache License 2.0)"
  echo "===================================================="
  echo ""
  echo "This directory (poc/freecad-webots-pipeline/) fetches, on demand, a small"
  echo "set of files from the ROBOTIS-GIT/turtlebot3 repository for use as a"
  echo "public, pre-rigged robot model in a FreeCAD-to-Webots pipeline spike."
  echo "These files are NOT committed to this repository; they are re-downloaded"
  echo "by 00_fetch_turtlebot3_assets.sh into the gitignored reference/ directory."
  echo ""
  echo "Source repository : $REPO_URL"
  echo "Pinned commit SHA  : $PINNED_SHA"
  echo "Retrieval date     : $RETRIEVAL_DATE"
  echo "License            : Apache License 2.0 (confirmed at pinned SHA)"
  echo ""
  echo "Files retrieved:"
  for rel_path in "${FETCHED_FILES[@]}"; do
    echo "  - $rel_path"
  done
  echo ""
  echo "Per Apache License 2.0 Section 4, this NOTICE file records the origin of"
  echo "the above third-party files. See $REPO_URL for the full LICENSE text at"
  echo "the pinned commit: $RAW_BASE/LICENSE"
} > "$NOTICE_FILE"
echo "  Wrote $NOTICE_FILE"

echo ""
echo "=================================================================="
echo "Stage 0 complete. Assets in: $REFERENCE_DIR"
echo "=================================================================="
