#!/usr/bin/env python3
"""
Tests for urdf_mesh_path_resolver.py (Issue #161)

Validates:
  - resolve_mesh_path() correctly resolves package:// URIs and relative paths
  - rewrite_for_webots() produces correct relative paths
  - rewrite_urdf_file() correctly rewrites mesh filenames in URDF XML
"""

import sys
import tempfile
from pathlib import Path
import pytest
import xml.etree.ElementTree as ET

# Script directory resolution
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

# Add script dir to path
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from urdf_mesh_path_resolver import resolve_mesh_path, rewrite_for_webots, rewrite_urdf_file


class TestResolveMeshPath:
    """Test resolve_mesh_path() function."""

    def test_resolve_package_uri_with_meshes_token(self):
        """Test resolving package:// URI with 'meshes' directory token."""
        urdf_dir = Path("/workspace/06_Exports/urdf")
        filename = "package://inverted_pendulum_robot/meshes/feetech-STS3032-visual.stl"

        # Should resolve to urdf_dir / meshes / feetech-STS3032-visual.stl
        result = resolve_mesh_path(filename, urdf_dir)
        assert result == (urdf_dir / "meshes" / "feetech-STS3032-visual.stl").resolve()

    def test_resolve_relative_path(self):
        """Test resolving relative path (no package:// prefix)."""
        urdf_dir = Path("/workspace/06_Exports/urdf")
        filename = "meshes/feetech-STS3032-visual.stl"

        result = resolve_mesh_path(filename, urdf_dir)
        assert result == (urdf_dir / filename).resolve()

    def test_resolve_package_uri_fallback_no_meshes_token(self):
        """Test resolving package:// URI without 'meshes' token (fallback)."""
        urdf_dir = Path("/workspace/06_Exports/urdf")
        filename = "package://inverted_pendulum_robot/subdir/file.stl"

        # Fallback: use last 2 components (subdir/file.stl)
        result = resolve_mesh_path(filename, urdf_dir)
        assert result == (urdf_dir / "subdir" / "file.stl").resolve()

    def test_resolve_short_package_uri_fallback(self):
        """Test fallback for very short package:// URI."""
        urdf_dir = Path("/workspace/06_Exports/urdf")
        filename = "package://robot/file.stl"

        # Fallback: use last 2 components (robot/file.stl)
        result = resolve_mesh_path(filename, urdf_dir)
        assert result == (urdf_dir / "robot" / "file.stl").resolve()


class TestRewriteForWebots:
    """Test rewrite_for_webots() function."""

    def test_rewrite_produces_relative_path(self):
        """Test that rewrite_for_webots() produces a relative path string."""
        # Create temporary directories to test path rewriting
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)

            # Create the expected mesh file
            urdf_dir = tmpdir / "06_Exports" / "urdf"
            meshes_dir = urdf_dir / "meshes"
            meshes_dir.mkdir(parents=True, exist_ok=True)
            mesh_file = meshes_dir / "test-visual.stl"
            mesh_file.touch()

            # Create webots directory
            webots_dir = tmpdir / "07_Simulation" / "webots" / ".generated"
            webots_dir.mkdir(parents=True, exist_ok=True)

            # Test rewriting
            filename = "package://inverted_pendulum_robot/meshes/test-visual.stl"
            result = rewrite_for_webots(filename, urdf_dir, webots_dir)

            # Result should be a relative path string
            assert isinstance(result, str)
            assert not result.startswith("/")  # Not absolute
            assert "test-visual.stl" in result

            # Verify the relative path is correct by resolving it
            resolved = (webots_dir / result).resolve()
            assert resolved == mesh_file.resolve()


class TestRewriteUrdfFile:
    """Test rewrite_urdf_file() function."""

    def test_rewrite_mesh_filenames_in_urdf(self):
        """Test that rewrite_urdf_file() correctly rewrites mesh filenames."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)

            # Create directory structure
            urdf_dir = tmpdir / "06_Exports" / "urdf"
            urdf_dir.mkdir(parents=True, exist_ok=True)
            meshes_dir = urdf_dir / "meshes"
            meshes_dir.mkdir(parents=True, exist_ok=True)

            # Create mesh files
            mesh1 = meshes_dir / "visual1.stl"
            mesh2 = meshes_dir / "visual2.stl"
            mesh1.touch()
            mesh2.touch()

            # Create source URDF with package:// URIs
            source_urdf = urdf_dir / "robot.urdf"
            source_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="link1">
    <visual>
      <mesh filename="package://test_robot/meshes/visual1.stl"/>
    </visual>
  </link>
  <link name="link2">
    <visual>
      <mesh filename="package://test_robot/meshes/visual2.stl"/>
    </visual>
  </link>
</robot>"""
            source_urdf.write_text(source_content)

            # Create webots directory
            webots_dir = tmpdir / "07_Simulation" / "webots" / ".generated"
            webots_dir.mkdir(parents=True, exist_ok=True)

            # Rewrite URDF
            output_urdf = webots_dir / "robot_webots.urdf"
            rewrite_urdf_file(source_urdf, output_urdf, urdf_dir, webots_dir)

            # Verify output file exists
            assert output_urdf.exists()

            # Parse output and check mesh filenames
            output_tree = ET.parse(output_urdf)
            output_root = output_tree.getroot()

            meshes = output_root.findall('.//mesh')
            assert len(meshes) == 2

            for mesh in meshes:
                filename = mesh.get('filename', '')
                # Should not be package:// URI
                assert not filename.startswith('package://')
                # Should be relative
                assert not filename.startswith('/')
                # Should resolve to actual file
                resolved = (webots_dir / filename).resolve()
                assert resolved.exists()

    def test_no_package_uris_remain_after_rewrite(self):
        """Test that no package:// URIs remain after rewriting."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)

            urdf_dir = tmpdir / "06_Exports" / "urdf"
            urdf_dir.mkdir(parents=True, exist_ok=True)
            meshes_dir = urdf_dir / "meshes"
            meshes_dir.mkdir(parents=True, exist_ok=True)

            mesh_file = meshes_dir / "visual.stl"
            mesh_file.touch()

            source_urdf = urdf_dir / "robot.urdf"
            source_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="link1">
    <visual>
      <mesh filename="package://test_robot/meshes/visual.stl"/>
    </visual>
  </link>
</robot>"""
            source_urdf.write_text(source_content)

            webots_dir = tmpdir / "07_Simulation" / "webots" / ".generated"
            webots_dir.mkdir(parents=True, exist_ok=True)

            output_urdf = webots_dir / "robot_webots.urdf"
            rewrite_urdf_file(source_urdf, output_urdf, urdf_dir, webots_dir)

            # Check output for package:// URIs
            output_content = output_urdf.read_text()
            assert 'package://' not in output_content


class TestEdgeCases:
    """Test edge cases and error conditions."""

    def test_input_file_not_found(self):
        """Test that rewrite_urdf_file() raises FileNotFoundError for missing input."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)
            nonexistent = tmpdir / "nonexistent.urdf"
            output = tmpdir / "output.urdf"

            with pytest.raises(FileNotFoundError):
                rewrite_urdf_file(nonexistent, output, tmpdir, tmpdir)

    def test_relative_path_string_from_rewrite_for_webots(self):
        """Test that rewrite_for_webots() returns a string (not a Path object)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)

            urdf_dir = tmpdir / "06_Exports" / "urdf"
            meshes_dir = urdf_dir / "meshes"
            meshes_dir.mkdir(parents=True, exist_ok=True)
            mesh_file = meshes_dir / "test.stl"
            mesh_file.touch()

            webots_dir = tmpdir / "07_Simulation" / "webots" / ".generated"
            webots_dir.mkdir(parents=True, exist_ok=True)

            result = rewrite_for_webots("meshes/test.stl", urdf_dir, webots_dir)
            assert isinstance(result, str)
            assert not isinstance(result, Path)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
