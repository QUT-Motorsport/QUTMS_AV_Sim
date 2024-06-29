from glob import glob
import os

from setuptools import setup

package_name = "sim_vehicle"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("meshes/*.dae")),
        (os.path.join("share", package_name), glob("urdf/*.urdf.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="URDF and mesh models for autonomous vehicles.",
    license="MIT",
    tests_require=["pytest"],
)
