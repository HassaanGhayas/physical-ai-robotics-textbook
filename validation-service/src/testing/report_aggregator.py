"""Test report aggregator service.

This module aggregates test results from all test suites into
comprehensive reports.
"""

import logging
from datetime import datetime
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import (
    TestSuiteResult,
    TestSuiteType,
    TestStatus,
)


logger = logging.getLogger(__name__)


class TestReportAggregator:
    """Service for aggregating test results into reports."""

    def __init__(self):
        """Initialize the report aggregator."""
        self.settings = get_settings()

    def aggregate(
        self,
        results: list[TestSuiteResult],
    ) -> dict:
        """Aggregate test results into a summary report.

        Args:
            results: List of test suite results

        Returns:
            Aggregated report dictionary
        """
        if not results:
            return self._empty_report()

        # Calculate totals
        total_tests = sum(r.total_tests for r in results)
        total_passed = sum(r.passed_count for r in results)
        total_failed = sum(r.failed_count for r in results)
        total_skipped = sum(r.skipped_count for r in results)
        total_errors = sum(r.error_count for r in results)

        # Calculate duration
        total_duration = sum(r.duration_seconds for r in results)

        # Determine overall status
        if total_errors > 0:
            overall_status = TestStatus.ERROR
        elif total_failed > 0:
            overall_status = TestStatus.FAILED
        elif total_tests == 0:
            overall_status = TestStatus.SKIPPED
        else:
            overall_status = TestStatus.PASSED

        # Collect flaky tests
        all_flaky = []
        for r in results:
            all_flaky.extend(r.flaky_tests_detected)

        # Calculate pass rate
        pass_rate = (total_passed / total_tests * 100) if total_tests > 0 else 0.0

        # Per-suite breakdown
        suite_breakdown = {}
        for r in results:
            suite_breakdown[r.suite_type.value] = {
                "total": r.total_tests,
                "passed": r.passed_count,
                "failed": r.failed_count,
                "skipped": r.skipped_count,
                "errors": r.error_count,
                "duration_seconds": r.duration_seconds,
                "status": r.status.value,
                "pass_rate": (
                    r.passed_count / r.total_tests * 100 if r.total_tests > 0 else 0.0
                ),
            }

        return {
            "summary": {
                "total_tests": total_tests,
                "passed": total_passed,
                "failed": total_failed,
                "skipped": total_skipped,
                "errors": total_errors,
                "duration_seconds": total_duration,
                "status": overall_status.value,
                "pass_rate": pass_rate,
            },
            "suites": suite_breakdown,
            "flaky_tests": all_flaky,
            "generated_at": datetime.utcnow().isoformat(),
        }

    def _empty_report(self) -> dict:
        """Create an empty report.

        Returns:
            Empty report dictionary
        """
        return {
            "summary": {
                "total_tests": 0,
                "passed": 0,
                "failed": 0,
                "skipped": 0,
                "errors": 0,
                "duration_seconds": 0.0,
                "status": TestStatus.SKIPPED.value,
                "pass_rate": 0.0,
            },
            "suites": {},
            "flaky_tests": [],
            "generated_at": datetime.utcnow().isoformat(),
        }

    def format_console(self, results: list[TestSuiteResult]) -> str:
        """Format results for console output.

        Args:
            results: Test suite results

        Returns:
            Formatted string for console
        """
        report = self.aggregate(results)
        summary = report["summary"]

        lines = [
            "",
            "=" * 60,
            "TEST EXECUTION SUMMARY",
            "=" * 60,
            "",
            f"Total Tests: {summary['total_tests']}",
            f"  Passed:  {summary['passed']}",
            f"  Failed:  {summary['failed']}",
            f"  Skipped: {summary['skipped']}",
            f"  Errors:  {summary['errors']}",
            "",
            f"Pass Rate: {summary['pass_rate']:.1f}%",
            f"Duration:  {summary['duration_seconds']:.2f}s",
            f"Status:    {summary['status'].upper()}",
            "",
        ]

        # Per-suite breakdown
        if report["suites"]:
            lines.extend([
                "-" * 60,
                "BY SUITE:",
                "",
            ])

            for suite_type, data in report["suites"].items():
                status_icon = "✓" if data["status"] == "passed" else "✗"
                lines.append(
                    f"  {suite_type.upper()}: {status_icon} "
                    f"{data['passed']}/{data['total']} passed "
                    f"({data['pass_rate']:.1f}%) "
                    f"in {data['duration_seconds']:.2f}s"
                )
            lines.append("")

        # Flaky tests
        if report["flaky_tests"]:
            lines.extend([
                "-" * 60,
                "FLAKY TESTS DETECTED:",
                "",
            ])
            for test_id in report["flaky_tests"]:
                lines.append(f"  ⚠ {test_id}")
            lines.append("")

        lines.append("=" * 60)
        return "\n".join(lines)

    def format_json(self, results: list[TestSuiteResult]) -> dict:
        """Format results as JSON.

        Args:
            results: Test suite results

        Returns:
            JSON-serializable dictionary
        """
        return self.aggregate(results)

    def format_html(self, results: list[TestSuiteResult]) -> str:
        """Format results as HTML.

        Args:
            results: Test suite results

        Returns:
            HTML string
        """
        report = self.aggregate(results)
        summary = report["summary"]

        status_class = "success" if summary["status"] == "passed" else "failure"

        html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Test Results</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .summary {{ background: #f5f5f5; padding: 20px; border-radius: 8px; }}
        .success {{ color: green; }}
        .failure {{ color: red; }}
        table {{ border-collapse: collapse; width: 100%; margin-top: 20px; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background: #4CAF50; color: white; }}
        tr:nth-child(even) {{ background: #f2f2f2; }}
    </style>
</head>
<body>
    <h1>Test Execution Results</h1>
    <div class="summary">
        <h2 class="{status_class}">{summary['status'].upper()}</h2>
        <p>Total Tests: {summary['total_tests']}</p>
        <p>Passed: {summary['passed']} | Failed: {summary['failed']} | Skipped: {summary['skipped']}</p>
        <p>Pass Rate: {summary['pass_rate']:.1f}%</p>
        <p>Duration: {summary['duration_seconds']:.2f}s</p>
    </div>

    <h2>Results by Suite</h2>
    <table>
        <tr>
            <th>Suite</th>
            <th>Status</th>
            <th>Total</th>
            <th>Passed</th>
            <th>Failed</th>
            <th>Pass Rate</th>
            <th>Duration</th>
        </tr>
"""

        for suite_type, data in report["suites"].items():
            html += f"""
        <tr>
            <td>{suite_type}</td>
            <td class="{'success' if data['status'] == 'passed' else 'failure'}">{data['status']}</td>
            <td>{data['total']}</td>
            <td>{data['passed']}</td>
            <td>{data['failed']}</td>
            <td>{data['pass_rate']:.1f}%</td>
            <td>{data['duration_seconds']:.2f}s</td>
        </tr>
"""

        html += """
    </table>
</body>
</html>
"""
        return html


def get_report_aggregator() -> TestReportAggregator:
    """Get report aggregator instance.

    Returns:
        TestReportAggregator instance
    """
    return TestReportAggregator()
