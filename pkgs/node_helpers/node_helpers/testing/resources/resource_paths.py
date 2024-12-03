from pathlib import Path


def resource_path(*paths: str | Path) -> Path:
    return Path(*paths).resolve(strict=True)
