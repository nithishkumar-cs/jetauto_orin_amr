"""Regression checks for the live Isaac Ethernet validation harness."""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from xml.etree import ElementTree

from scripts.validation.run_isaac_wired_validation import write_fastdds_profile


class FastDdsProfileTest(unittest.TestCase):
    """Keep Isaac Ethernet and local safety-node discovery available."""

    def test_profile_allows_ethernet_and_loopback(self):
        """The generated transport whitelist includes both required interfaces."""
        with TemporaryDirectory() as directory:
            profile = Path(directory) / "fastdds.xml"
            write_fastdds_profile(profile, "169.254.71.212")
            root = ElementTree.parse(profile).getroot()
            namespace = {"fastdds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
            addresses = {
                element.text
                for element in root.findall(".//fastdds:interfaceWhiteList/fastdds:address", namespace)
            }
            self.assertEqual(addresses, {"169.254.71.212", "127.0.0.1"})


if __name__ == "__main__":
    unittest.main()
