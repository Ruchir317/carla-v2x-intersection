import os
import sys


def ensure_carla_path():
    """Make CARLA's `agents` package importable.

    It ships in the CARLA install (PythonAPI/carla), not in the pip package. Point CARLA_ROOT at the
    install directory, or add PythonAPI/carla to PYTHONPATH yourself."""
    root = os.environ.get("CARLA_ROOT")
    if root:
        path = os.path.join(root, "PythonAPI", "carla")
        if os.path.isdir(path) and path not in sys.path:
            sys.path.append(path)
