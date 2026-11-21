"""Shared SLC2 recovery for the standalone tool and both navigation drivers."""
from concurrent.futures import Future, TimeoutError
from queue import Empty, Full, Queue
import struct


RECOVERY_ARMED = 0x10
RECOVERY_CAPABLE = 0x40
RECOVERY_ALLOWED = 0x80
TEST_LOCK = 0x20
RECOVERABLE_FAULTS = 0x01 | 0x10 | TEST_LOCK
MOTOR_POWER_CONFIGURED = 0x38
RECOVERY_TELEMETRY_TIMEOUT = .3
MAX_TELEMETRY_GAP = 3.


def fault_mask(body_status):
    return (body_status or 0) >> 8


def recoverable(body_status):
    mask = fault_mask(body_status)
    return bool(body_status and body_status & RECOVERY_CAPABLE and mask
                and not mask & ~RECOVERABLE_FAULTS)


def paired_body_status(body_status, wheel_tick, body_tick):
    if (body_status or 0) & RECOVERY_CAPABLE and wheel_tick != body_tick:
        return None
    return body_status


def request_recovery(car, operation=0):
    car.send(0x19, struct.pack('<IB', car.nonce, operation))


class LinkRecovery:
    def __init__(self, arm_watchdog=False):
        self.active = False
        self.arm_watchdog = arm_watchdog
        self.next_arm = 0.
        self.capability_seen = False
        self.need_body_after = float('-inf')
        self.next_request = 0.
        self.requests = Queue(maxsize=1)
        self.test_in_progress = None
        self.test_deadline = 0.

    def queue_test(self):
        result = Future()
        try:
            self.requests.put_nowait(result)
        except Full:
            result.set_result((False, 'A lock test is already queued'))
        return result

    def start_test_service(self, rospy):
        from std_srvs.srv import Trigger, TriggerResponse

        def handle(_request):
            result = self.queue_test()
            try:
                return TriggerResponse(*result.result(timeout=2.))
            except TimeoutError:
                result.cancel()
                return TriggerResponse(False, 'No confirmed MCU lock telemetry')

        return rospy.Service('~test_chassis_lock', Trigger, handle)

    def wait_for_telemetry(self, now, body_status, wheel_age):
        if (not self.arm_watchdog or not (body_status or 0) & RECOVERY_CAPABLE
                or wheel_age >= MAX_TELEMETRY_GAP):
            return False
        self.active = True
        self.need_body_after = now
        return True

    def update(self, car, now, flags, body_status, *, last_body_status=None):
        """Return (hold zero output, recovered this cycle) using fresh telemetry."""
        supported = bool(body_status and body_status & RECOVERY_CAPABLE)
        self.capability_seen |= supported or bool((last_body_status or 0) & RECOVERY_CAPABLE)
        ready = (supported and flags & MOTOR_POWER_CONFIGURED == MOTOR_POWER_CONFIGURED
                 and not flags & 0xc0 and body_status & 7 == 7)
        fault = bool(flags & 4 or fault_mask(body_status))
        armed = bool(body_status and body_status & RECOVERY_ARMED)
        if self.arm_watchdog and ready and not fault and not armed and now >= self.next_arm:
            request_recovery(car, operation=2)
            self.next_arm = now + .25
        try:
            test = self.requests.get_nowait()
        except Empty:
            test = None
        if test is not None and test.set_running_or_notify_cancel():
            if not ready or fault or self.active or (self.arm_watchdog and not armed):
                test.set_result((False, 'C30D must be healthy, powered, enabled and on recovery-capable firmware'))
            else:
                request_recovery(car, operation=1)
                self.active = True
                self.test_in_progress = test
                self.test_deadline = now + 1.
                return True, False

        if self.test_in_progress is not None:
            if flags & 4 and fault_mask(body_status) & TEST_LOCK:
                self.test_in_progress.set_result((True, 'MCU test lock confirmed. Automatic recovery follows.'))
                self.test_in_progress = None
            elif now >= self.test_deadline:
                self.test_in_progress.set_result((False, 'MCU did not confirm test lock'))
                self.test_in_progress = None
            else:
                return True, False

        if fault:
            if self.capability_seen:
                self.active = True
            if (ready and flags & 4 and recoverable(body_status)
                    and body_status & RECOVERY_ALLOWED and now >= self.next_request):
                request_recovery(car)
                self.next_request = now + .25
            return self.active, False
        if self.active and ready:
            self.active = False
            return True, True
        return self.active, False
