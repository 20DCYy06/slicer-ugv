"""Desktop command session that keeps the same Runtime across pick and release."""
from concurrent.futures import ThreadPoolExecutor
import logging
import os
import select
import signal
import sys
import time


def run(runtime, pick_on_start):
    try:
        _run_session(runtime, pick_on_start)
    except Exception as exc:
        logging.getLogger('visual_grasp').exception('后台抓取意外退出')
        print('[ERROR]', exc, file=sys.stderr)
        state = 'recovery' if runtime.needs_release else 'error'
        print('GRASP_STATE ' + state, flush=True)
        raise SystemExit(1) from exc


def _run_session(runtime, pick_on_start):
    logger = logging.getLogger('visual_grasp')
    stopping = False
    camera_started = False

    def exit_signal(signum, frame):
        nonlocal stopping
        stopping = True

    def report(state):
        print('GRASP_STATE '+state, flush=True)

    def execute(command):
        nonlocal camera_started
        try:
            if command == b'pick':
                if not camera_started:
                    runtime.camera.start()
                    camera_started = True
                # The graphical launch has a running preview before the operator
                # clicks. Wait for camera startup only, then use the same action.
                deadline = time.monotonic() + 5.
                while True:
                    runtime.check_cancelled()
                    frame, _, _, error = runtime.camera.snapshot()
                    if frame is not None and not error:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(error or '等待相机启动超时')
                    runtime.cancel.wait(.05)
                runtime.pick()
                return 'held'
            runtime.release()
            return 'idle'
        except InterruptedError:
            runtime.log('[STOP] 动作已中断')
        except Exception as exc:
            logger.exception('任务失败')
            runtime.log('[ERROR] '+str(exc))
        return 'recovery' if runtime.needs_release or command == b'release' else 'error'

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, exit_signal)

    fd = sys.stdin.fileno()
    pending = b''
    runtime.load_recovery()
    with ThreadPoolExecutor(max_workers=1, thread_name_prefix='snapshot-grasp') as worker:
        try:
            future = None
            state = 'recovery' if runtime.needs_release else 'idle'
            if pick_on_start and state == 'idle' and not stopping:
                report('busy')
                future = worker.submit(execute, b'pick')
            else:
                report(state)

            while not stopping:
                if select.select([fd], [], [], .05)[0]:
                    chunk = os.read(fd, 4096)
                    if not chunk or stopping:
                        break
                    *commands, pending = (pending + chunk).split(b'\n')
                    for command in commands:
                        if command not in (b'pick', b'release'):
                            runtime.log('[ERROR] 未知抓取指令')
                        elif future is not None:
                            runtime.log('[ERROR] 当前抓取或释放尚未结束')
                        elif command == b'pick' and state in ('held', 'recovery'):
                            runtime.log('[ERROR] 请先释放')
                        else:
                            report('busy')
                            future = worker.submit(execute, command)
                    if len(pending) > 4096:
                        raise ValueError('抓取指令过长')

                if future is not None and future.done() and not stopping:
                    state = future.result()
                    future = None
                    report(state)
        finally:
            runtime.cancel.set()
            runtime.camera.close()
    if runtime.needs_release:
        report('recovery')
    elif future is not None:
        report('idle')
