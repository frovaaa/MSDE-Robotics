from glob import glob

from setuptools import setup

package_name = "usi_angry_turtle"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/action", glob("action/*.action")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="davide",
    maintainer_email="frovad@usi.ch",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move2goal_node = usi_angry_turtle.move2goal_node:main",
            "writer_node = usi_angry_turtle.writer_node:main",
        ],
    },
)
