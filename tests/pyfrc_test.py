"""
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
"""

import pytest

@pytest.mark.filterwarnings("ignore")
def test_lazy(control: "TestController"): # type: ignore
    with control.run_robot():
        control.step_timing(seconds=1, autonomous=True, enabled=False)
        control.step_timing(seconds=1, autonomous=True, enabled=True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=False)
        control.step_timing(seconds=1, autonomous=False, enabled=True)
