#!/usr/bin/python3
"""Start RViz with explicit map sampler bindings for this OpenGL 4.5 host.

Ogre validates the indexed map shader before applying material uniforms; its
sampler1D and sampler2D otherwise both default to unit zero. Use a process-local
resource overlay instead of changing the system ROS installation.
"""
import hashlib
import os
from pathlib import Path
import shutil
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    source = Path(get_package_share_directory("rviz_rendering"))
    shader = Path("ogre_media/materials/glsl120/indexed_8bit_image.frag")
    original = (source / shader).read_text()
    patched = original.replace(
        "#version 120", "#version 120\n#extension GL_ARB_shading_language_420pack : require", 1
    ).replace(
        "uniform sampler2D eight_bit_image;",
        "layout(binding = 0) uniform sampler2D eight_bit_image;"
    ).replace(
        "uniform sampler1D palette;", "layout(binding = 1) uniform sampler1D palette;"
    )
    if patched == original or "layout(binding = 1)" not in patched:
        raise RuntimeError("RViz indexed-map shader has changed; compatibility overlay needs updating")
    digest = hashlib.sha256((str(source) + patched).encode()).hexdigest()[:16]
    cache = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache")))
    prefix = cache / "hetero_fleet" / ("rviz_rendering_" + digest)
    target = prefix / "share/rviz_rendering"
    if not (target / shader).exists():
        shutil.copytree(source, target, dirs_exist_ok=True)
        (target / shader).write_text(patched)
        marker = prefix / "share/ament_index/resource_index/packages/rviz_rendering"
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
    env = dict(os.environ)
    env["AMENT_PREFIX_PATH"] = str(prefix) + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
    executable = str(Path(get_package_prefix("rviz2")) / "lib/rviz2/rviz2")
    os.execve(executable, [executable, *sys.argv[1:]], env)


if __name__ == "__main__":
    main()
