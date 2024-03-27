from setuptools import find_packages, setup
import os
from glob import glob

package_name = "steward_description"

setup(
    name=package_name,
    version="0.0.0",  # See package.xml
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob(os.path.join("meshes", "*.stl")),
        ),
        (
            os.path.join("share", package_name, "urdf/configs"),
            glob(os.path.join("urdf/configs", "*"), recursive=True),
        ),
        (
            os.path.join("share", package_name, "urdf/accessories"),
            glob(os.path.join("urdf/accessories", "*"), recursive=True),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.*")),
        ),
        ("share/" + package_name, ["urdf/empty.urdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheitman",
    maintainer_email="will@heit.mn",  # See package.xml
    description="TODO: Package description",  # See package.xml
    license="TODO: License declaration",  # See package.xml
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
