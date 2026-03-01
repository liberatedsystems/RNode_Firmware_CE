import unittest
from unittest.mock import MagicMock, patch
import sys
import os

# Add Python Module to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../Python Module')))

from RNode import RNodeInterface, KISS

class TestRNodeInterface(unittest.TestCase):
    @patch('serial.Serial')
    def test_init(self, mock_serial):
        callback = MagicMock()
        rnode = RNodeInterface(callback, "TestRNode", "/dev/ttyUSB0", frequency=868000000, bandwidth=125000, txpower=10, sf=7, cr=5)
        
        self.assertTrue(rnode.online)
        mock_serial.assert_called()

    @patch('serial.Serial')
    def test_invalid_config(self, mock_serial):
        callback = MagicMock()
        with self.assertRaises(ValueError):
            RNodeInterface(callback, "TestRNode", "/dev/ttyUSB0", frequency=0, bandwidth=125000, txpower=10, sf=7, cr=5)

if __name__ == '__main__':
    unittest.main()
