from setuptools import find_packages, setup

package_name = "racecar_beacon"
executables = [
    "lab_chat_server",
    "lab_chat_client",
    "lab_poll_pos",
    "remote_client",
    "ros_monitor",
    "vehicle_tracker",
]

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marc-Olivier Fecteau",
    maintainer_email="fecm0701@usherbrooke.ca",
    description="Package for GRO520-related features",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"{executable} = {package_name}.{executable}:main"
            for executable in executables
        ]
    },
)
