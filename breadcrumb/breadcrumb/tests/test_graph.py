"""Tests for resolve_name_params() and expand_for_each_param() in breadcrumb.graph."""

import warnings

import pytest

from breadcrumb.graph import expand_for_each_param, resolve_name_params
from breadcrumb.node_interface import ParameterDefinition


def _make_param(name: str, default_value=None, param_type: str = "string") -> ParameterDefinition:
    return ParameterDefinition(name=name, param_type=param_type, default_value=default_value)


class TestResolveNameParams:
    """Tests for resolve_name_params()."""

    def test_no_param_reference(self):
        """Names without ${param:...} are returned unchanged."""
        assert resolve_name_params("/cmd_vel", {}, {}) == "/cmd_vel"

    def test_single_partial_substitution(self):
        """A single ${param:...} embedded in a name is resolved."""
        result = resolve_name_params(
            "/robot/${param:id}/cmd",
            {"id": "robot1"},
            {},
        )
        assert result == "/robot/robot1/cmd"

    def test_multiple_substitutions(self):
        """Multiple ${param:...} references in one name are all resolved."""
        result = resolve_name_params(
            "/${param:ns}/${param:id}/data",
            {"ns": "fleet", "id": "unit3"},
            {},
        )
        assert result == "/fleet/unit3/data"

    def test_full_substitution(self):
        """A name that is entirely a ${param:...} reference is resolved."""
        result = resolve_name_params(
            "${param:topic}",
            {"topic": "/my_topic"},
            {},
        )
        assert result == "/my_topic"

    def test_launch_param_priority_over_default(self):
        """Launch parameters take priority over interface defaults."""
        result = resolve_name_params(
            "/robot/${param:id}/cmd",
            {"id": "launch_value"},
            {"id": _make_param("id", default_value="default_value")},
        )
        assert result == "/robot/launch_value/cmd"

    def test_falls_back_to_interface_default(self):
        """Falls back to interface default when no launch param is provided."""
        result = resolve_name_params(
            "/robot/${param:id}/cmd",
            {},
            {"id": _make_param("id", default_value="default_robot")},
        )
        assert result == "/robot/default_robot/cmd"

    def test_integer_param_auto_converted(self):
        """Integer default values are converted to strings."""
        result = resolve_name_params(
            "/robot/${param:port}/data",
            {},
            {"port": _make_param("port", default_value=42, param_type="integer")},
        )
        assert result == "/robot/42/data"

    def test_whitespace_tolerance(self):
        """Whitespace around the parameter name is tolerated."""
        result = resolve_name_params(
            "/robot/${param: robot_id }/cmd",
            {"robot_id": "r1"},
            {},
        )
        assert result == "/robot/r1/cmd"

    def test_unresolvable_left_as_is(self):
        """Unresolvable references are left as-is in the output."""
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            result = resolve_name_params(
                "/robot/${param:missing}/cmd",
                {},
                {},
            )
        assert result == "/robot/${param:missing}/cmd"

    def test_unresolvable_emits_warning(self):
        """Unresolvable references emit a UserWarning."""
        with pytest.warns(UserWarning, match="missing"):
            resolve_name_params(
                "/robot/${param:missing}/cmd",
                {},
                {},
            )

    def test_mixed_resolved_and_unresolved(self):
        """One param resolves, another doesn't — partial resolution."""
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            result = resolve_name_params(
                "/${param:ns}/${param:missing}/data",
                {"ns": "found"},
                {},
            )
        assert result == "/found/${param:missing}/data"

    def test_non_param_syntax_unchanged(self):
        """Non-param ${...} syntax (e.g., ${env:HOME}) is left untouched."""
        result = resolve_name_params(
            "/robot/${env:HOME}/cmd",
            {},
            {},
        )
        assert result == "/robot/${env:HOME}/cmd"

    def test_none_default_falls_through(self):
        """A parameter with default_value=None is treated as unresolvable."""
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            result = resolve_name_params(
                "/robot/${param:id}/cmd",
                {},
                {"id": _make_param("id", default_value=None)},
            )
        assert result == "/robot/${param:id}/cmd"


class TestExpandForEachParam:
    """Tests for expand_for_each_param()."""

    def test_no_for_each_param(self):
        """Names without ${for_each_param:...} return a single-element list."""
        result = expand_for_each_param("/cmd_vel", {}, {})
        assert result == ["/cmd_vel"]

    def test_no_for_each_param_with_param_ref(self):
        """Names with only ${param:...} are resolved and returned as single-element list."""
        result = expand_for_each_param(
            "/robot/${param:id}/cmd",
            {"id": "robot1"},
            {},
        )
        assert result == ["/robot/robot1/cmd"]

    def test_basic_expansion_from_launch_params(self):
        """Basic expansion from a launch parameter list."""
        result = expand_for_each_param(
            "/cameras/${for_each_param:camera_names}/image",
            {"camera_names": ["front", "rear", "left"]},
            {},
        )
        assert result == [
            "/cameras/front/image",
            "/cameras/rear/image",
            "/cameras/left/image",
        ]

    def test_expansion_from_interface_defaults(self):
        """Expansion using interface parameter default values."""
        result = expand_for_each_param(
            "/sensors/${for_each_param:sensor_list}/data",
            {},
            {"sensor_list": _make_param("sensor_list", default_value=["lidar", "radar"], param_type="string_array")},
        )
        assert result == [
            "/sensors/lidar/data",
            "/sensors/radar/data",
        ]

    def test_launch_params_priority_over_defaults(self):
        """Launch parameters take priority over interface defaults."""
        result = expand_for_each_param(
            "/topic/${for_each_param:names}/out",
            {"names": ["alpha", "beta"]},
            {"names": _make_param("names", default_value=["x", "y", "z"], param_type="string_array")},
        )
        assert result == [
            "/topic/alpha/out",
            "/topic/beta/out",
        ]

    def test_coexistence_with_param_ref(self):
        """${for_each_param:...} and ${param:...} coexist in the same name."""
        result = expand_for_each_param(
            "/${param:ns}/${for_each_param:items}/data",
            {"ns": "fleet", "items": ["cam1", "cam2"]},
            {},
        )
        assert result == [
            "/fleet/cam1/data",
            "/fleet/cam2/data",
        ]

    def test_single_element_array(self):
        """A single-element array produces one result."""
        result = expand_for_each_param(
            "/topic/${for_each_param:names}/out",
            {"names": ["only"]},
            {},
        )
        assert result == ["/topic/only/out"]

    def test_empty_array(self):
        """An empty array produces an empty list."""
        result = expand_for_each_param(
            "/topic/${for_each_param:names}/out",
            {"names": []},
            {},
        )
        assert result == []

    def test_non_list_param_warns_and_falls_back(self):
        """A non-list parameter emits a warning and returns single-element fallback."""
        with pytest.warns(UserWarning, match="not a list"):
            result = expand_for_each_param(
                "/topic/${for_each_param:names}/out",
                {"names": "not_a_list"},
                {},
            )
        assert len(result) == 1

    def test_missing_param_warns_and_falls_back(self):
        """A missing parameter emits a warning and returns single-element fallback."""
        with pytest.warns(UserWarning, match="Could not resolve for_each_param"):
            result = expand_for_each_param(
                "/topic/${for_each_param:missing}/out",
                {},
                {},
            )
        assert len(result) == 1

    def test_whitespace_tolerance(self):
        """Whitespace around the parameter name in the token is tolerated."""
        result = expand_for_each_param(
            "/topic/${for_each_param: names }/out",
            {"names": ["a", "b"]},
            {},
        )
        assert result == ["/topic/a/out", "/topic/b/out"]
