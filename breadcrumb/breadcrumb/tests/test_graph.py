"""Tests for resolve_name_params() in breadcrumb.graph."""

import warnings

import pytest

from breadcrumb.graph import resolve_name_params
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
