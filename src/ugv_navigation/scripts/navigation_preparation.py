"""ROS client of the shared field navigation preparation operation."""
import json
import threading
import time


RUNTIME_TOPIC = '/geographic/navigation_runtime'
PREPARE_ACTION = '/geographic/prepare'


class PreparationClient:
    def __init__(self, ros, string_type, action_name):
        self.ros, self.action_name = ros, action_name
        self.status = None
        self.client = None
        self.phase = ('FAILED', 'no preparation request')
        self.lock = threading.RLock()
        self.subscription = ros.Subscriber(RUNTIME_TOPIC, string_type, self._status, queue_size=1)

    def _status(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.status = (time.monotonic(), value)
        except (TypeError, ValueError):
            pass

    def observation(self):
        sample = self.status
        if sample and time.monotonic() - sample[0] <= 1. and sample[1].get('action') == self.action_name:
            return sample[1]
        return {}

    def start(self, task_id=None, target=None):
        import actionlib
        from ugv_navigation.msg import PrepareGeographicAction, PrepareGeographicGoal

        if self.client is None:
            self.client = actionlib.SimpleActionClient(PREPARE_ACTION, PrepareGeographicAction)
        with self.lock:
            self.phase = ('WAITING_NAVIGATION', 'contacting shared navigation owner')
            self.pending = PrepareGeographicGoal(task_id=str(task_id or ''),
                                                 target_json=json.dumps(target, allow_nan=False))
            self.deadline = time.monotonic() + 4.
            self.sent = False
            self.sent_at = None

    def _done(self, status, result):
        from actionlib_msgs.msg import GoalStatus

        with self.lock:
            if result and result.phase:
                self.phase = result.phase, result.reason
            elif status == GoalStatus.RECALLED:
                self.phase = 'STOPPED', 'preparation canceled before admission'
            else:
                self.phase = 'FAILED', 'preparation Action connection lost'

    def _feedback(self, result):
        with self.lock:
            if self.phase[0] not in ('READY', 'FAILED', 'STOPPED'):
                self.phase = result.phase, result.reason

    def poll(self):
        with self.lock:
            pending = getattr(self, 'pending', None)
        if pending is not None:
            available = self.client.wait_for_server(self.ros.Duration(.01))
            with self.lock:
                if self.pending is not None and available:
                    self.client.send_goal(self.pending, done_cb=self._done, feedback_cb=self._feedback)
                    self.pending, self.sent = None, True
                    self.sent_at = time.monotonic()
                elif self.pending is not None and time.monotonic() >= self.deadline:
                    self.pending = None
                    self.phase = ('FAILED', 'shared navigation preparation Action unavailable')
        with self.lock:
            if (getattr(self, 'sent_at', None) is not None
                    and self.phase[0] not in ('READY', 'FAILED', 'STOPPED')
                    and time.monotonic() - self.sent_at > 3. and not self.observation()):
                self.client.cancel_goal()
                self.phase = ('FAILED', 'shared navigation owner status lost; stop unconfirmed')
            return self.phase

    def cancel(self):
        with self.lock:
            if getattr(self, 'pending', None) is not None:
                self.pending = None
                self.phase = ('STOPPED', 'preparation canceled before sending')
            elif getattr(self, 'sent', False) and self.phase[0] not in ('READY', 'FAILED', 'STOPPED'):
                self.client.cancel_goal()
