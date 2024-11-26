import numpy as np
import numpy.typing as npt
import tf_transformations as transformations
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3

from . import numpify
from .registry import converts_from_numpy, converts_to_numpy

# basic types


@converts_to_numpy(Vector3)
def vector3_to_numpy(msg: Vector3, hom: bool = False) -> npt.NDArray[np.float64]:
    if hom:
        return np.array([msg.x, msg.y, msg.z, 0])
    else:
        return np.array([msg.x, msg.y, msg.z])


@converts_from_numpy(Vector3)
def numpy_to_vector3(arr: npt.NDArray[np.float64]) -> Vector3:
    if arr.shape[-1] == 4:
        assert np.all(arr[..., -1] == 0)
        arr = arr[..., :-1]

    if len(arr.shape) == 1:
        return Vector3(**dict(zip(["x", "y", "z"], arr, strict=False)))
    else:
        return np.apply_along_axis(
            lambda v: Vector3(**dict(zip(["x", "y", "z"], v, strict=False))),
            axis=-1,
            arr=arr,
        )


@converts_to_numpy(Point)
def point_to_numpy(msg: Point, hom: bool = False) -> npt.NDArray[np.float64]:
    if hom:
        return np.array([msg.x, msg.y, msg.z, 1])
    else:
        return np.array([msg.x, msg.y, msg.z])


@converts_from_numpy(Point)
def numpy_to_point(arr: npt.NDArray[np.float64]) -> Point:
    if arr.shape[-1] == 4:
        arr = arr[..., :-1] / arr[..., -1]

    if len(arr.shape) == 1:
        return Point(**dict(zip(["x", "y", "z"], arr, strict=True)))
    else:
        return np.apply_along_axis(
            lambda v: Point(**dict(zip(["x", "y", "z"], v, strict=True))),
            axis=-1,
            arr=arr,
        )


@converts_to_numpy(Quaternion)
def quat_to_numpy(msg: Quaternion) -> npt.NDArray[np.float64]:
    return np.array([msg.x, msg.y, msg.z, msg.w])


@converts_from_numpy(Quaternion)
def numpy_to_quat(arr: npt.NDArray[np.float64]) -> Quaternion:
    assert arr.shape[-1] == 4

    if len(arr.shape) == 1:
        return Quaternion(**dict(zip(["x", "y", "z", "w"], arr, strict=False)))
    else:
        return np.apply_along_axis(
            lambda v: Quaternion(**dict(zip(["x", "y", "z", "w"], v, strict=False))),
            axis=-1,
            arr=arr,
        )


# compound types
# all of these take ...x4x4 homogeneous matrices


@converts_to_numpy(Transform)
def transform_to_numpy(msg: Transform) -> npt.NDArray[np.float64]:
    return np.dot(
        transformations.translation_matrix(numpify(msg.translation)),
        transformations.quaternion_matrix(numpify(msg.rotation)),
    )


@converts_from_numpy(Transform)
def numpy_to_transform(arr: npt.NDArray[np.float64]) -> Transform:
    shape, rest = arr.shape[:-2], arr.shape[-2:]
    assert rest == (4, 4)

    if len(shape) == 0:
        trans = transformations.translation_from_matrix(arr)
        quat = transformations.quaternion_from_matrix(arr)

        return Transform(
            translation=Vector3(**dict(zip(["x", "y", "z"], trans, strict=True))),
            rotation=Quaternion(**dict(zip(["x", "y", "z", "w"], quat, strict=True))),
        )
    else:
        res = np.empty(shape, dtype=np.object_)
        for idx in np.ndindex(shape):
            res[idx] = Transform(
                translation=Vector3(
                    **dict(
                        zip(
                            ["x", "y", "z"],
                            transformations.translation_from_matrix(arr[idx]),
                            strict=True,
                        )
                    )
                ),
                rotation=Quaternion(
                    **dict(
                        zip(
                            ["x", "y", "z", "w"],
                            transformations.quaternion_from_matrix(arr[idx]),
                            strict=True,
                        )
                    )
                ),
            )


@converts_to_numpy(Pose)
def pose_to_numpy(msg: Pose) -> npt.NDArray[np.float64]:
    return np.dot(
        transformations.translation_matrix(numpify(msg.position)),
        transformations.quaternion_matrix(numpify(msg.orientation)),
    )


@converts_from_numpy(Pose)
def numpy_to_pose(arr: npt.NDArray[np.float64]) -> Pose:
    shape, rest = arr.shape[:-2], arr.shape[-2:]
    assert rest == (4, 4)

    if len(shape) == 0:
        trans = transformations.translation_from_matrix(arr)
        quat = transformations.quaternion_from_matrix(arr)

        return Pose(
            position=Point(**dict(zip(["x", "y", "z"], trans, strict=False))),
            orientation=Quaternion(
                **dict(zip(["x", "y", "z", "w"], quat, strict=False))
            ),
        )
    else:
        res = np.empty(shape, dtype=np.object_)
        for idx in np.ndindex(shape):
            res[idx] = Pose(
                position=Point(
                    **dict(
                        zip(
                            ["x", "y", "z"],
                            transformations.translation_from_matrix(arr[idx]),
                            strict=True,
                        )
                    )
                ),
                orientation=Quaternion(
                    **dict(
                        zip(
                            ["x", "y", "z", "w"],
                            transformations.quaternion_from_matrix(arr[idx]),
                            strict=True,
                        )
                    )
                ),
            )
