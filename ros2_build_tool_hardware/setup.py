"""
Setup configuration for ros2_build_tool_hardware
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="ros2_build_tool_hardware",
    version="0.2.0",
    author="ROS2 Build Tool Contributors",
    description="Hardware discovery, URDF parsing, and ros2_control integration",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/jlbucklew/ros2-build-tool",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Build Tools",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.8",
    install_requires=[
        "ros2_build_tool_core>=0.2.0",
        "pyyaml>=5.1",
    ],
)