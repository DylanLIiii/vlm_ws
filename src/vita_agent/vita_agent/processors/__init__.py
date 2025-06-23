"""
Processors package for VITA Agent.

This package contains various processors for handling different types of data
and commands in the VITA agent system.
"""

from .sensor_processor import SensorProcessor
from .asr_processor import ASRProcessor

__all__ = ['SensorProcessor', 'ASRProcessor']