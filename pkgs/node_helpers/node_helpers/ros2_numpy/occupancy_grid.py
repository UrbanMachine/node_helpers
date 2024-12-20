from array import array

import numpy as np
import numpy.typing as npt
from nav_msgs.msg import MapMetaData, OccupancyGrid

from .registry import converts_from_numpy, converts_to_numpy


@converts_to_numpy(OccupancyGrid)
def occupancygrid_to_numpy(msg: OccupancyGrid) -> npt.NDArray[np.float64]:
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    return np.ma.array(data, mask=data == -1, fill_value=-1)


@converts_from_numpy(OccupancyGrid)
def numpy_to_occupancy_grid(
    arr: npt.NDArray[np.float64], info: MapMetaData = None
) -> OccupancyGrid:
    if not len(arr.shape) == 2:
        raise TypeError("Array must be 2D")
    if not arr.dtype == np.int8:
        raise TypeError("Array must be of int8s")

    grid = OccupancyGrid()
    if isinstance(arr, np.ma.MaskedArray):
        # We assume that the masked value are already -1, for speed
        arr = arr.data

    grid.data = array("b", arr.ravel().astype(np.int8))
    grid.info = info or MapMetaData()
    grid.info.height = arr.shape[0]
    grid.info.width = arr.shape[1]

    return grid
