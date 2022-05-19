#!/usr/bin/env python

import sys
from os import path, walk
from pathlib import Path
from setuptools import setup, find_packages
from setuptools.command.build_py import build_py


package_name = "mujoco_utils"
package_version = "1.0.0"


class custom_build_py(build_py):
    """Build the documentation prior to the package.

    Args:
        build_py (class): inherit from the build_py class which builds the current
                     python package.
    """
    def run(self):

        # Try to build the doc and install it.
        try:
            # Get the mpi_cmake_module build doc method
            from mpi_cmake_modules.documentation_builder import (
                build_documentation,
            )

            build_documentation(
                str(
                    (
                        Path(self.build_lib) / package_name / "doc"
                    ).absolute()
                ),
                str(Path(__file__).parent.absolute()),
                package_version,
            )
        except ImportError as e:
            print("The documentation is not being built as the "
                  "mpi_cmake_modules is not found", file=sys.stderr)

        # distutils uses old-style classes, so no super()
        build_py.run(self)


def find_resources(package_name):
    """ Find the relative path of files under the resource folder. """
    resources = []
    package_dir = path.join("src", package_name)
    resources_dir = path.join(package_dir, "resources")

    for (root, _, files) in walk(resources_dir):
        for afile in files:
            if (
                afile != package_name
                and not afile.endswith(".DS_Store")
                and not afile.endswith(".py")
            ):
                rel_dir = path.relpath(root, package_dir)
                src = path.join(rel_dir, afile)
                resources.append(src)
    return resources


with open(path.join(path.dirname(path.realpath(__file__)), "readme.md"), "r") as fh:
    long_description = fh.read()

# Find the resource files.
resources = find_resources(package_name)

# Install nodes and examples.
scripts_list = []
for (root, _, files) in walk(path.join("examples")):
    for demo_file in files:
        if demo_file.endswith(".py"):
            scripts_list.append(path.join(root, demo_file))

# Setup the package
setup(
    name=package_name,
    version="1.0.0",
    package_dir={'': 'src',},
    packages=find_packages(where='src'),
    package_data={package_name: resources},
    scripts=scripts_list,
    # required dependencies.
    install_requires=["setuptools", "dm_control", "importlib_resources"],
    # optional (hack) dependencies.
    tests_require=["pytest", "mpi_cmake_modules"],
    #
    zip_safe=True,
    maintainer="Julian Viereck",
    maintainer_email="jv1439@nyu.edu",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pypa/sampleproject",
    description="Wrapper around the pybullet interface using pinocchio.",
    license="BSD-3-clause",
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    cmdclass={"build_py": custom_build_py},
)
