"""
Setup configuration for ros2_build_tool_watchdog
"""

from setuptools import setup, find_packages

setup(
    name="ros2_build_tool_watchdog",
    version="0.3.0-alpha",
    author="ROS2 Build Tool Contributors",
    description="Self-healing watchdog for ROS2 topic monitoring",
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
        "rclpy",
    ],
    entry_points={
        'console_scripts': [
            'topic_watchdog = ros2_build_tool_watchdog.topic_watchdog:main',
        ],
    },
)