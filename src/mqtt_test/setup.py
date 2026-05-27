from setuptools import find_packages, setup

package_name = "mqtt_test"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="parichu",
    maintainer_email="parichu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mqtt_pub = mqtt_test.pub_cmd_vel:main",
            "nav_node = mqtt_test.nav:main",
            "mq_pub = mqtt_test.mq_pub:main",
        ],
    },
)
