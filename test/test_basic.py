#!/usr/bin/env python3
import pathlib
import unittest
from system_fingerprint.imprint import main


class TestBasic(unittest.TestCase):
    def test_basic_smoke(self):
        """Run main, hope for no errors"""
        main([])

        output_file = pathlib.Path('fingerprint.yaml')
        if output_file.exists():
            output_file.unlink()


if __name__ == '__main__':
    import rostest
    rostest.rosrun('system_fingerprint', 'test_smoke', TestBasic)
