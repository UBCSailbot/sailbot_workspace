from os.path import join

from setuptools import setup

PACKAGE_NAME = "global_launch"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=[],
    install_requires=["setuptools"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (join("share", PACKAGE_NAME), ["main_launch.py"]),
    ],
    zip_safe=True,
    maintainer="UBC Sailbot",
    maintainer_email="software@ubcsailbot.org",
    description="UBC Sailbot's global launch file",
    license="MIT",
)
