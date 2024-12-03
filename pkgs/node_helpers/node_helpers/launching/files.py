from pathlib import Path


def required_directory(*elements: str | Path) -> Path:
    path = Path(*elements)
    if not path.is_dir():
        raise FileNotFoundError(f"Directory {path} not found")
    return path


def required_file(*elements: str | Path) -> Path:
    path = Path(*elements)
    if not path.is_file():
        raise FileNotFoundError(f"File {path} not found")
    return path
