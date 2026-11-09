from setuptools import setup, find_packages

package_name = "rob_box_dialog"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    # Каталог фраз (ADR-0150 §5.4): colcon в образе ставит без --symlink-install — без этой строки YAML в
    # install-дерево не попадёт и ``rob_box_dialog.phrases`` не загрузится.
    # Основа промпта Личности и нейтральная персона (ADR-0167 Ш1б-3) - так же, данные пакета.
    package_data={"rob_box_dialog.phrases": ["*.yaml"], "rob_box_dialog.prompts": ["*.txt"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=False,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="Pure-Python dialog v2 core (ADR-0150): knowledge table, word matching",
    license="MIT",
    tests_require=["pytest"],
)
