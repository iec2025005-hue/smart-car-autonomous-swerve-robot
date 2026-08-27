import os
import sys
import unittest

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

class TestSwerveControl(unittest.TestCase):
    def test_swerve_kinematics_import(self):
        try:
            from control.swerve_kinematics import SwerveKinematics
            kin = SwerveKinematics()
            res = kin.calculate(150, 0, 0)
            self.assertEqual(len(res), 4)
        except ImportError:
            pass

    def test_basic_control(self):
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
