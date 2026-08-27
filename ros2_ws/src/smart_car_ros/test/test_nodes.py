import os
import sys
import unittest

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

class TestSmartCarRos(unittest.TestCase):
    def test_navigation_state_machine(self):
        from navigation.navigator import Navigator
        from vision.detector import Detection

        nav = Navigator()
        self.assertEqual(nav.state, "IDLE")
        
        nav.set_mode("search")
        nav.update([])
        self.assertEqual(nav.state, "SEARCH")

        det = Detection(label="target", confidence=0.95, bbox=(280, 80, 60, 160), center=(310, 160))
        nav.set_mode("follow")
        cmd = nav.update([det])
        self.assertIn(nav.state, ["TARGET_DETECTED", "APPROACH"])

if __name__ == '__main__':
    unittest.main()
