from node_helpers.parameters import param_path


def test_basic_operation() -> None:
    result = param_path("pkgs/node_helpers/node_helpers/__init__.py")
    assert result.is_file()
