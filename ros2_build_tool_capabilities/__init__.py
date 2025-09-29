"""
ROS2 Build Tool Capabilities Package

Provides high-level capability abstractions for robot functionality,
enabling novice-friendly configuration without requiring deep ROS2 knowledge.
"""

from .capability_manager import CapabilityManager, Capability
from .capability_registry import CapabilityRegistry, CapabilityDefinition
from .capability_wizard import CapabilityWizard

__all__ = [
    'CapabilityManager',
    'Capability',
    'CapabilityRegistry',
    'CapabilityDefinition',
    'CapabilityWizard'
]

__version__ = '0.5.0'