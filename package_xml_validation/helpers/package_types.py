from __future__ import annotations

import os
import re
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from lxml.etree import _Element

    # Project-wide alias for an lxml element. Centralizing it here means the
    # rest of the codebase imports `XmlElement` instead of
    # `lxml.etree._Element`, so swapping the underlying type (e.g. wrapping
    # it in a domain class) only requires editing this line.
    XmlElement = _Element


class PackageType(Enum):
    CMAKE_PKG = "ament_cmake"
    PYTHON_PKG = "ament_python"
    MSG_PKG = "rosidl_default_generators"


def get_package_type(xml_file: str) -> tuple[PackageType, bool]:
    """Determine package type based on files and message-generation usage.

    Args:
        xml_file: Path to a package.xml file.

    Returns:
        A tuple of (PackageType, is_msg_pkg).

    """
    cmake_file = os.path.join(os.path.dirname(xml_file), "CMakeLists.txt")
    setup_file = os.path.join(os.path.dirname(xml_file), "setup.py")
    is_msg_pkg = False
    pkg_type = PackageType.CMAKE_PKG
    if os.path.exists(cmake_file):
        regex = r"rosidl_generate_interfaces\s*\(\s*.*?\)"
        with open(cmake_file) as f:
            content = f.read()
            if re.search(regex, content, re.DOTALL):
                is_msg_pkg = True
        pkg_type = PackageType.CMAKE_PKG
    elif os.path.exists(setup_file):
        pkg_type = PackageType.PYTHON_PKG
    return pkg_type, is_msg_pkg
