from pathlib import Path


def param_path(path_str: str | Path) -> Path:
    """Interprets the provided path string as relative to the root of the repository if
    it is a relative path.

    :param path_str: The path provided as a ROS parameter
    :raises RuntimeError: If the code is placed in some strange location
    :return: An absolute path
    """

    path = Path(path_str)
    if path.is_absolute():
        return path

    # Find the root of the repository
    root_candidate = Path.cwd()
    while not (root_candidate / "pkgs").is_dir():
        if len(root_candidate.parents) == 0:
            raise RuntimeError(
                f"Could not find root of repository! Started at '{Path.cwd()}'."
            )
        root_candidate = root_candidate.parent

    return root_candidate / path
