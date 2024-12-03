from tempfile import NamedTemporaryFile, TemporaryDirectory

import pytest
from node_helpers.launching import required_directory, required_file


def test_required_file() -> None:
    with pytest.raises(FileNotFoundError):
        required_file("/i_dont_exist")

    with NamedTemporaryFile() as file_:
        required_file(file_.name)


def test_required_directory() -> None:
    with pytest.raises(FileNotFoundError):
        required_directory("/seriously_this_dir_isnt_real")

    with TemporaryDirectory() as dir_:
        required_directory(dir_)
