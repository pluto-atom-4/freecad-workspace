#!/usr/bin/env python3
"""
Shared Pipeline Logging Utility (Issue #166)

Provides centralized log_event() function for all phases (10, 12, 13) to write to
a unified audit trail at: /inverted-pendulum-project/.generated/pipeline_audit.log

All three phases append to the same log file with phase markers to maintain a
complete pipeline execution history for debugging and verification.

Usage:
    from pipeline_logger import log_event
    log_event("Phase 10", "Axis validation started")  # or:
    log_event("Phase 12", "URDF validation passed")
    log_event("Phase 13", "PROTO structure check failed")
"""

import sys
from pathlib import Path
from datetime import datetime, timezone


def get_shared_log_path() -> Path:
    """
    Get the shared audit log path: /inverted-pendulum-project/.generated/pipeline_audit.log

    Returns:
        Path to the shared audit log file
    """
    # Find the project root by locating inverted-pendulum-project directory
    try:
        script_file = Path(__file__).resolve()
    except NameError:
        # Fallback if __file__ is not available
        script_file = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "pipeline_logger.py"

    # If this file is in the project root, use that directly
    if script_file.parent.name == "inverted-pendulum-project":
        project_root = script_file.parent
    else:
        # Otherwise search up the directory tree for inverted-pendulum-project
        current = script_file.parent
        while current != current.parent:  # Stop at filesystem root
            if current.name == "inverted-pendulum-project":
                project_root = current
                break
            current = current.parent
        else:
            # Fallback to home-based path if not found in tree
            project_root = Path.home() / "freecad-workspace" / "inverted-pendulum-project"

    return project_root / ".generated" / "pipeline_audit.log"


def log_event(phase: str, message: str) -> None:
    """
    Log an event to the shared pipeline audit log.

    Args:
        phase: Phase identifier (e.g., "Phase 10", "Phase 12", "Phase 13")
        message: Event message to log

    Raises:
        None - failures are silently logged to stderr only
    """
    log_file = get_shared_log_path()

    # Ensure .generated directory exists
    log_file.parent.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S')
    log_line = f"[{timestamp}] [{phase}] {message}"

    try:
        with open(log_file, 'a') as f:
            f.write(log_line + '\n')
    except Exception as e:
        print(f"Warning: Failed to write to shared log file ({log_file}): {e}", file=sys.stderr)
