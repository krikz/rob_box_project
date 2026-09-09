from setuptools import setup, find_packages

package_name = "rob_box_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # issue #2199: единый конфиг сегментации речи для всех голосовых
        # путей. Используется и Python-вызывающими (rob_box_quest),
        # и генератором TS-констант для webxr_client.
        ("share/" + package_name + "/config", ["config/speech_segmentation.yaml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="Shared cross-node abstractions for rob_box harness architecture (ADR-0001 P0 foundation)",
    license="MIT",
    tests_require=["pytest"],
)
