"""
Setup configuration for ros2_build_tool_generators
"""

from setuptools import setup, find_packages

setup(
    name="ros2_build_tool_generators",
    version="0.2.0",
    author="ROS2 Build Tool Contributors",
    description="Launch file, configuration, and parameter generation with lifecycle support",
    url="https://github.com/jlbucklew/ros2-build-tool",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Build Tools",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
    ],
    python_requires=">=3.8",
    install_requires=[
        "ros2_build_tool_core>=0.2.0",
    ],
)