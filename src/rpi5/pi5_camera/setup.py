from setuptools import find_packages, setup

package_name = "pi5_camera"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robobike maintainers",
    maintainer_email="maintainers@example.com",
    description="USB camera image publisher for Raspberry Pi 5.",
    license="MIT",
    entry_points={"console_scripts": ["camera_node = pi5_camera.camera_node:main"]},
)
