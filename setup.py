from glob import glob
from setuptools import find_packages, setup


package_name = "astra_rgb_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="guyuchuan",
    maintainer_email="guyuchuan@todo.todo",
    description="Standalone V4L2 RGB publisher for the Orbbec Astra Pro Plus.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgb_camera_node = astra_rgb_camera.rgb_camera_node:main",
        ],
    },
)
