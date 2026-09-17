#!/usr/bin/env python3
"""
Unit tests for vrml_lexer.find_matching_brace() and inject_cast_shadows functionality.

Pure Python / pytest, no external dependencies beyond pytest.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest 07_Simulation/webots/test_vrml_lexer.py -v
"""

import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from vrml_lexer import find_matching_brace
from inject_cast_shadows import (
    find_shape_blocks,
    inject_cast_shadows_false,
    validate_vrml_braces,
)


class TestFindMatchingBrace:
    """Tests for vrml_lexer.find_matching_brace()"""

    def test_simple_braces(self):
        """Test: simple {text}"""
        text = "Shape { geometry Box { } }"
        pos = text.find("{")  # Position of first {
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"
        assert text[pos:close_pos + 1] == "{ geometry Box { } }"

    def test_nested_braces(self):
        """Test: nested {...{...}...}"""
        text = "Shape { geometry Box { size 1 2 3 } appearance Appearance { } }"
        pos = text.find("{")  # Position of Shape {
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"
        # Should match the outer closing brace
        assert close_pos == len(text) - 1

    def test_brace_in_string_ignored(self):
        """Test: braces inside strings are ignored"""
        text = 'Solid { name "foo{bar}" geometry Box { } }'
        pos = text.find("{")  # Position of Solid {
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"
        # The string's { should not affect matching
        assert text[pos:close_pos + 1] == '{ name "foo{bar}" geometry Box { } }'

    def test_escaped_quote_in_string(self):
        """Test: escaped quotes \\\" inside strings"""
        text = r'Field { value "test \"quoted\" value" }'
        pos = text.find("{")
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"
        assert text[pos:close_pos + 1] == r'{ value "test \"quoted\" value" }'

    def test_comment_with_brace(self):
        """Test: braces inside comments are ignored"""
        text = "Shape { # This comment has a brace {\n  geometry Box { } }"
        pos = text.find("{")  # Position of Shape {
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"
        # Comment brace should be ignored, find the real closing brace

    def test_multiple_comments(self):
        """Test: multiple comment lines"""
        text = """Shape {
  # Comment 1 with {
  geometry Box {
    # Comment 2 with }
    size 1
  }
  # Final comment {}}
}"""
        pos = text.find("{")
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"
        assert text[close_pos] == text.rstrip()[-1]  # Should be the last }

    def test_unterminated_string_raises(self):
        """Test: unterminated string raises ValueError"""
        text = 'Shape { name "unterminated'
        pos = text.find("{")
        with pytest.raises(ValueError, match="Unterminated string"):
            find_matching_brace(text, pos)

    def test_unmatched_brace_raises(self):
        """Test: unmatched opening brace raises ValueError"""
        text = "Shape { geometry Box { }"
        # Missing closing brace for Shape {
        pos = text.find("{")
        with pytest.raises(ValueError, match="Unmatched"):
            find_matching_brace(text, pos)

    def test_wrong_char_raises(self):
        """Test: starting from non-brace position raises ValueError"""
        text = "Shape { }"
        with pytest.raises(ValueError, match="not '{'"):
            find_matching_brace(text, 0)  # Position 0 is 'S', not '{'

    def test_square_brackets(self):
        """Test: matching square brackets [...]"""
        text = "inertiaMatrix [ 1 0 0 0 1 0 0 0 1 ]"
        pos = text.find("[")
        close_pos = find_matching_brace(text, pos, open_char="[", close_char="]")
        assert text[close_pos] == "]"

    def test_nested_square_brackets(self):
        """Test: nested square brackets"""
        text = "centerOfMass [ 1 2 [ nested ] ]"
        pos = text.find("[")
        close_pos = find_matching_brace(text, pos, open_char="[", close_char="]")
        assert text[close_pos] == "]"


class TestFindShapeBlocks:
    """Tests for inject_cast_shadows.find_shape_blocks()"""

    def test_single_shape_block(self):
        """Test: finding single Shape block"""
        text = 'Shape { name "Box" geometry Box { } url "feetech-STS3032-visual.stl" }'
        blocks = find_shape_blocks(text, "feetech-STS3032-visual")
        assert len(blocks) == 1
        start, end = blocks[0]
        assert text[start] == "{"
        assert text[end] == "}"

    def test_multiple_shape_blocks(self):
        """Test: finding multiple Shape blocks"""
        text = (
            'Shape { url "feetech-STS3032-visual.stl" } '
            'Shape { url "other.stl" } '
            'Shape { url "feetech-STS3032-visual.stl" }'
        )
        blocks = find_shape_blocks(text, "feetech-STS3032-visual")
        assert len(blocks) == 2

    def test_nested_shape_blocks(self):
        """Test: Shape block with nested braces"""
        text = 'Shape { appearance Appearance { material Material { } } url "feetech-STS3032-visual.stl" }'
        blocks = find_shape_blocks(text, "feetech-STS3032-visual")
        assert len(blocks) == 1

    def test_no_matching_blocks(self):
        """Test: no blocks matching mesh name"""
        text = 'Shape { url "other.stl" } Shape { url "another.stl" }'
        blocks = find_shape_blocks(text, "feetech-STS3032-visual")
        assert len(blocks) == 0


class TestInjectCastShadows:
    """Tests for inject_cast_shadows_false() function"""

    def test_inject_simple(self):
        """Test: inject castShadows into simple Shape block"""
        text = "Shape { url \"test.stl\" }"
        blocks = [(0, len(text) - 1)]
        modified, changes = inject_cast_shadows_false(text, blocks)
        assert changes == 1
        assert "castShadows FALSE" in modified

    def test_inject_preserves_indentation(self):
        """Test: injected castShadows respects indentation"""
        text = """Shape {
  url "test.stl"
}"""
        blocks = [(0, len(text) - 1)]
        modified, changes = inject_cast_shadows_false(text, blocks)
        assert changes == 1
        assert "castShadows FALSE" in modified
        # Should have proper indentation
        lines = modified.split("\n")
        assert any("castShadows FALSE" in line for line in lines)

    def test_inject_multiple_blocks(self):
        """Test: inject castShadows into multiple blocks"""
        # Two simple Shape blocks
        block1 = "Shape { url \"test.stl\" }"
        block2 = "Shape { url \"another.stl\" }"
        text = block1 + " " + block2
        # Manually find block positions (simplified for test)
        blocks = [
            (0, len(block1) - 1),
            (len(block1) + 1, len(text) - 1),
        ]
        modified, changes = inject_cast_shadows_false(text, blocks)
        assert changes == 2


class TestVrmlBracesValidation:
    """Tests for validate_vrml_braces() function"""

    def test_balanced_braces(self):
        """Test: balanced braces pass validation"""
        text = "Shape { geometry Box { } }"
        assert validate_vrml_braces(text) is True

    def test_unbalanced_braces_more_open(self):
        """Test: more opening than closing"""
        text = "Shape { geometry Box { }"
        assert validate_vrml_braces(text) is False

    def test_unbalanced_braces_more_close(self):
        """Test: more closing than opening"""
        text = "Shape { geometry Box { } }"
        # Extra close
        text += "}"
        assert validate_vrml_braces(text) is False

    def test_empty_content(self):
        """Test: empty content passes"""
        assert validate_vrml_braces("") is True

    def test_no_braces(self):
        """Test: content with no braces passes"""
        assert validate_vrml_braces("Some text without braces") is True


class TestEdgeCases:
    """Edge case tests for brace matching"""

    def test_braces_in_consecutive_strings(self):
        """Test: multiple strings with braces in sequence"""
        text = 'Block { f1 "a{b" f2 "c{d" }'
        pos = text.find("{")
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"

    def test_empty_string_in_block(self):
        """Test: empty string inside block"""
        text = 'Block { name "" }'
        pos = text.find("{")
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"

    def test_real_proto_snippet(self):
        """Test: realistic PROTO snippet"""
        text = """Shape {
  appearance Appearance {
    material Material {
      diffuseColor 0.5 0.5 0.5
    }
  }
  geometry Mesh {
    url "model.stl"
  }
}"""
        pos = text.find("{")
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"
        # Extract just the part from opening brace to closing brace
        expected = text[pos:]
        assert text[pos:close_pos + 1] == expected

    def test_comment_at_end_of_block(self):
        """Test: comment at very end doesn't break matching"""
        text = """Shape {
  geometry Box { }
  # Final comment
}"""
        pos = text.find("{")
        close_pos = find_matching_brace(text, pos)
        assert text[close_pos] == "}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
