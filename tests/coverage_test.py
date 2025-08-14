import unittest
import os
import sys

if os.getenv("COVERAGE", "1") == "1":
    import coverage

    cov = coverage.Coverage(source=["package_xml_validation"])
    cov.start()

suite = unittest.defaultTestLoader.discover("tests")
result = unittest.TextTestRunner(verbosity=2).run(suite)

if os.getenv("COVERAGE", "1") == "1":
    cov.stop()
    cov.save()
    cov.report(show_missing=True)
    cov.xml_report(outfile="coverage.xml")
    cov.html_report(directory="htmlcov")

sys.exit(0 if result.wasSuccessful() else 1)
