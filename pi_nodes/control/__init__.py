"""Pure-Python control policies extracted from motor_node for testability.

These classes hold their own state but never touch MQTT, GPIO, or hardware
directly — call sites pass in observations and receive decisions back. That
keeps the safety-critical loop in a single process (MQTT round-trip would
add 10-50 ms to collision response, unacceptable) while letting each policy
be unit-tested in isolation.
"""
