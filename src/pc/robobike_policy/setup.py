from setuptools import find_packages, setup

package_name = "robobike_policy"

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
    description="ROS adapter for externally hosted SmolVLA inference.",
    license="MIT",
    entry_points={"console_scripts": ["policy_node = robobike_policy.policy_node:main"]},
)
