from pathlib import Path

import numpy as np
import numpy.typing as npt

from .resource_paths import resource_path


class NumpyResource:
    """A helper class for loading *.npy files"""

    def __init__(self, *paths: str | Path):
        self.path = resource_path(*paths)

    @property
    def array(self) -> npt.DTypeLike:
        with self.path.open("rb") as npy_file:
            array: npt.DTypeLike = np.load(npy_file)
        return array
