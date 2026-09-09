from setuptools import find_packages, setup
from glob import glob
import os

package_name = "rob_box_telegram"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "rob_box_core>=0.1.0"],
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="Telegram bot operator interface for Rob Box autonomous rover",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telegram_node = rob_box_telegram.telegram_node:main",
        ],
    },
)
