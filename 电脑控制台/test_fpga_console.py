import time
import tkinter as tk
import unittest
from unittest.mock import Mock

from fpga_console import App
from protocol import CALIBRATE, DRIVE, FIRMWARE, GO, GOAL, POINT, ROUTE


class BoardNavigationTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.addCleanup(self.root.destroy)
        self.root.withdraw()
        self.app = App(self.root, enumerate_ports=False)
        self.app.connected = self.app.remote_ready = True
        self.app.last_task = time.monotonic()
        self.app.att = {'ready': True}
        self.app.sensors = {'IMU_INIT': True}
        self.app.task = dict(
            firmware=FIRMWARE, imu_fresh=True, manual_range_fresh=True,
            calibrated=True, pose=True, can_go=True, width_cm=200,
            height_cm=200, active=False,
        )
        self.app.link.submit = Mock()

    def test_calibrate_sends_board_command(self):
        self.app.calibrate_frame()
        self.app.link.submit.assert_called_once_with(
            [(DRIVE, 0, 0, 0), (CALIBRATE, 0, 0, 0)])

    def test_goal_and_start_send_no_pc_motion_command(self):
        self.app.goal_x.set('70')
        self.app.goal_y.set('90')
        self.app.goal(True)
        self.app.link.submit.assert_called_once_with(
            [(DRIVE, 0, 0, 0), (GOAL, 700, 900, 0), (GO, 0, 0, 0)])
        self.assertIsNone(self.app.link.thread)

    def test_start_requires_board_readiness(self):
        self.app.task['can_go'] = False
        self.app.goal(True)
        self.app.link.submit.assert_not_called()

    def test_four_points_use_board_route(self):
        self.app.route(True)
        operations = [command[0] for command in self.app.link.submit.call_args.args[0]]
        self.assertEqual(operations, [DRIVE, POINT, POINT, POINT, POINT, ROUTE])
