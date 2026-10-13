"""Unit tests for observer-side map transform timing."""

import unittest

from analyze_map_tf_coverage import analyze_samples


class AnalyzeMapTfCoverageTest(unittest.TestCase):
    def test_counts_immediate_delayed_and_missing_coverage(self):
        scans = [(1.0, 0.8), (2.0, 1.8), (3.0, 4.0)]
        transforms = [(0.5, 1.0), (2.2, 2.0), (3.2, 3.0)]
        result = analyze_samples(scans, transforms)
        self.assertEqual(result["covered_at_scan_receipt"], 1)
        self.assertEqual(result["uncovered_at_end"], 1)
        self.assertEqual(result["wait_until_covering_tf_receipt_s"]["count"], 2)
        self.assertAlmostEqual(result["wait_until_covering_tf_receipt_s"]["median"], 0.1)

    def test_no_tf_at_receipt(self):
        result = analyze_samples([(0.5, 1.0)], [(1.0, 1.1)])
        self.assertEqual(result["no_map_tf_at_scan_receipt"], 1)
        self.assertEqual(result["covered_at_scan_receipt"], 0)
        self.assertEqual(result["uncovered_at_end"], 0)
        self.assertAlmostEqual(result["wait_until_covering_tf_receipt_s"]["median"], 0.5)

    def test_first_transform_receipt_for_source_scan(self):
        result = analyze_samples(
            [(1.0, 0.5)], [(1.25, 0.7), (1.5, 0.7)],
            localizer_transform_timeout_s=0.2,
        )
        self.assertEqual(result["first_tf_receipt_minus_source_scan_receipt_wall_s"]["count"], 1)
        self.assertAlmostEqual(
            result["first_tf_receipt_minus_source_scan_receipt_wall_s"]["median"], 0.25
        )


if __name__ == "__main__":
    unittest.main()
