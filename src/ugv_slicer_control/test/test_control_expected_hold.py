#!/usr/bin/env python3
"""Optional client-side expected hold binds a task RESUME across process startup."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import control


class ExpectedHoldTests(unittest.TestCase):
    def test_expected_hold_check_is_available(self):
        self.assertTrue(callable(getattr(control, 'check_expected_hold', None)))

    @unittest.skipUnless(hasattr(control, 'check_expected_hold'), 'not implemented yet')
    def test_only_exact_instance_and_generation_can_release(self):
        state = dict(mux_instance_id='owned', takeover_generation=7)
        control.check_expected_hold(state, 'owned', 7)
        for instance, generation in [('new-instance', 7), ('owned', 8), ('owned', None), (None, 7)]:
            with self.subTest(instance=instance, generation=generation):
                with self.assertRaises(ValueError):
                    control.check_expected_hold(state, instance, generation)

    @unittest.skipUnless(hasattr(control, 'check_expected_hold'), 'not implemented yet')
    def test_ordinary_operator_cli_has_unchanged_default(self):
        control.check_expected_hold(dict(mux_instance_id='new', takeover_generation=100), None, None)


if __name__ == '__main__':
    unittest.main()
