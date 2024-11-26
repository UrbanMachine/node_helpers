from collections.abc import Sequence
from typing import Any

_to_numpy = {}
_from_numpy = {}


def converts_to_numpy(msgtype: Any, plural: bool = False) -> Any:
    def decorator(f: Any) -> Any:
        _to_numpy[msgtype, plural] = f
        return f

    return decorator


def converts_from_numpy(msgtype: Any, plural: bool = False) -> Any:
    def decorator(f: Any) -> Any:
        _from_numpy[msgtype, plural] = f
        return f

    return decorator


def numpify(msg: Any, *args: Any, **kwargs: Any) -> Any:
    if msg is None:
        return None

    conv = _to_numpy.get((msg.__class__, False))
    if not conv and isinstance(msg, Sequence):
        if not msg:
            raise ValueError("Cannot determine the type of an empty Collection")
        conv = _to_numpy.get((msg[0].__class__, True))

    if not conv:
        raise ValueError(
            "Unable to convert message {} - only supports {}".format(
                msg.__class__.__name__,
                ", ".join(cls.__name__ + ("[]" if pl else "") for cls, pl in _to_numpy),
            )
        )

    return conv(msg, *args, **kwargs)


def msgify(msg_type: Any, numpy_obj: Any, *args: Any, **kwargs: Any) -> Any:
    conv = _from_numpy.get((msg_type, kwargs.pop("plural", False)))
    if not conv:
        raise ValueError(
            "Unable to build message {} - only supports {}".format(
                msg_type.__name__,
                ", ".join(cls.__name__ + ("[]" if pl else "") for cls, pl in _to_numpy),
            )
        )
    return conv(numpy_obj, *args, **kwargs)
