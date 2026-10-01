from setuptools import setup, find_packages

package_name = "rob_box_music"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="Pure-Python music model (Track, validator, knowledge table) for the arranger v2 (ADR-0149)",
    license="MIT",
    tests_require=["pytest"],
)
