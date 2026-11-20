import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
SOURCES=Path(__file__).resolve().parents[2]
class PatrolPause(unittest.TestCase):
    def run_pause(self, wait_for_chassis, localization_lost=False):
        sys.path.insert(0, str(SOURCES/'slicer_navigation/scripts'))
        import multi_goal_patrol as module
        import yaml
        patrol = module.Patrol.__new__(module.Patrol)
        patrol.p = yaml.safe_load((SOURCES/'slicer_navigation/config/patrol_params.yaml').read_text())
        patrol.p.update(wait_for_localization=localization_lost, wait_for_chassis_recovery=wait_for_chassis)
        clock = dict(now=0.)
        states, commands = [], []
        patrol.active = False
        patrol.localized = not localization_lost
        patrol.running = lambda: clock['now'] < 25.
        patrol.chassis_ready = lambda: localization_lost or clock['now'] < 1. or clock['now'] >= 21.
        patrol.pose = lambda: (1. if clock['now'] >= 21. else 0., 0., 0.)
        patrol.state = states.append
        patrol.spin_pub = patrol.stop_pub = SimpleNamespace(publish=commands.append)
        patrol.cancel_active_goal = lambda: setattr(patrol, 'active', False)
        patrol.client = SimpleNamespace(send_goal=lambda goal: None,
                                       get_state=lambda: 1 if patrol.active else 2)
        patrol.goal = lambda point: point
        patrol.plan_topics, patrol.planner_events = {}, []
        patrol.plan_count = lambda *args: 20
        patrol.align_heading = lambda *args: (True, 0.)
        fake_time = SimpleNamespace(monotonic=lambda: clock['now'],
                                    sleep=lambda seconds: clock.update(now=clock['now']+seconds))
        fake_ros = SimpleNamespace(loginfo=lambda *a: None, logwarn=lambda *a: None,
                                   logerr=lambda *a: None, loginfo_throttle=lambda *a: None)
        with patch.object(module, 'time', fake_time), patch.object(module, 'rospy', fake_ros):
            result = patrol.navigate(dict(name='test', x=1., y=0., yaw=0.))
        self.assertTrue(all(m.linear.x == 0 and m.angular.z == 0 for m in commands))
        return result, states, clock['now']

    def test_online_chassis_pause_can_exceed_15_seconds_and_then_complete(self):
        result, states, elapsed = self.run_pause(True)
        self.assertTrue(result)
        self.assertGreater(elapsed, 20.)
        self.assertIn('PAUSED test', states)

    def test_default_patrol_keeps_its_original_pause_timeout(self):
        result, _, elapsed = self.run_pause(False)
        self.assertFalse(result)
        self.assertLess(elapsed, 20.)

    def test_localization_loss_still_uses_existing_timeout(self):
        result, _, elapsed = self.run_pause(True, localization_lost=True)
        self.assertFalse(result)
        self.assertLess(elapsed, 20.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
