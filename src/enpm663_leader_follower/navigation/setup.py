from setuptools import setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer=["sajjad", "carissa", "shail"],
    maintainer_email=["yasin@umd.edu", "carillo@umd.edu", "sshah115@umd.edu"],
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "leader_navigator = navigation.leader_navigator_interface:main",
            "follower_navigator = navigation.follower_navigator_interface:main",
        ],
    },
)
