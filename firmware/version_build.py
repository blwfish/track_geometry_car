"""
PlatformIO build script for automatic version injection.

Injects:
- BUILD_GIT_HASH: Short git commit hash (e.g., "af66a81")
- BUILD_TIME: Build timestamp (e.g., "2025-01-02 15:30")
- BUILD_DIRTY: "+" if working directory has uncommitted changes

The firmware version string becomes: "0.1.0-af66a81" or "0.1.0-af66a81+"
"""

import subprocess
from datetime import datetime

Import("env")

def get_git_hash():
    """Get short git commit hash."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, cwd=env.get("PROJECT_DIR")
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return "unknown"

def get_git_dirty():
    """Check if working directory has uncommitted changes."""
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            capture_output=True, text=True, cwd=env.get("PROJECT_DIR")
        )
        if result.returncode == 0 and result.stdout.strip():
            return True
    except Exception:
        pass
    return False

def get_build_time():
    """Get current build timestamp."""
    return datetime.now().strftime("%Y-%m-%d %H:%M")

# Get version info
git_hash = get_git_hash()
git_dirty = "+" if get_git_dirty() else ""
build_time = get_build_time()

print(f"Build version: {git_hash}{git_dirty} at {build_time}")

# Inject as build flags
env.Append(CPPDEFINES=[
    ("BUILD_GIT_HASH", f'\\"{git_hash}{git_dirty}\\"'),
    ("BUILD_TIME", f'\\"{build_time}\\"'),
])
