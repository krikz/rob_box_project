#!/usr/bin/env python3
"""
Setup file для Animation Editor
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="rob_box_animation_editor",
    version="1.0.0",
    author="rob_box team",
    description="LED Animation Editor для rob_box робота",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "Pillow>=8.0.0",
        "PyYAML>=5.4.0",
    ],
    entry_points={
        "console_scripts": [
            "animation-editor=main:main",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Multimedia :: Graphics :: Editors",
    ],
)
