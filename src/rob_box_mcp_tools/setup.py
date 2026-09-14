from setuptools import setup, find_packages

package_name = 'rob_box_mcp_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    # Issue #1998 §6.2 — slice_policy.yaml живёт ВНУТРИ пакета
    # (``rob_box_mcp_tools/data/``) и читается через
    # ``importlib.resources.files("rob_box_mcp_tools.data")``, а не через
    # ament share. Значит доставка — package_data, а не data_files:
    # прод-образ собирается БЕЗ ``--symlink-install``
    # (docker/vision/voice_assistant/Dockerfile:239), то есть пакет реально
    # устанавливается, и без этой строки .yaml в install-дерево не попадает.
    # Последствие пропуска — ``load_default_authority()`` бросает ConfigError,
    # mcp_server стартует с пустой политикой и блокирует ВСЕ инструменты.
    # Регрессия закрыта тестом test_slice_policy_packaging.py.
    package_data={
        'rob_box_mcp_tools.data': ['*.yaml', 'rtttl_melodies.jsonl.gz'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # Пакет читает свои ресурсы с диска (slice_policy.yaml) — из zip-egg
    # это работает не на всех путях установки, поэтому явно не zip-safe.
    zip_safe=False,
    maintainer='Rob Box Team',
    maintainer_email='ros2@rob-box.local',
    description='MCP-like tool system for LLM integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcp_server = rob_box_mcp_tools.mcp_server:main',
        ],
    },
)
