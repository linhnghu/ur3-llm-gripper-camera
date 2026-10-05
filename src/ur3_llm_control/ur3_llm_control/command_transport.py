"""Shared transport settings for language command input and task status."""
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

COMMAND_TOPIC = "/llm_command"
STATUS_TOPIC = "/llm_status"


def status_qos():
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
