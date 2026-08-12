"""`--exclude-package`: a package the validator leaves alone entirely."""

import shutil
import tempfile
import unittest
from pathlib import Path

from package_xml_validation.package_xml_validator import PackageXmlValidator


def _write_package_xml(path: Path, name: str, body: str = ""):
    """A manifest whose elements are deliberately out of schema order, so every one of these
    packages has something the validator would want to rewrite."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <description>Demo package</description>
  <version>0.0.0</version>
  <license>MIT</license>
  {body}
</package>
""",
        encoding="utf-8",
    )


class TestExcludePackage(unittest.TestCase):
    def setUp(self):
        self.tmpdir = Path(tempfile.mkdtemp(prefix="exclude_package_"))
        # Names that share a prefix, which is the case the directory-matching version of this
        # would have got wrong.
        for name in ("demo_pkg", "demo_pkg_extra"):
            _write_package_xml(self.tmpdir / name / "package.xml", name)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def manifest(self, name):
        return self.tmpdir / name / "package.xml"

    def validator(self, **kwargs):
        return PackageXmlValidator(check_rosdeps=False, **kwargs)

    def test_an_excluded_package_is_not_rewritten(self):
        before = self.manifest("demo_pkg").read_bytes()

        self.validator(excluded_packages=["demo_pkg"]).check_and_format(
            [str(self.tmpdir)]
        )

        self.assertEqual(before, self.manifest("demo_pkg").read_bytes())

    def test_a_sibling_sharing_the_prefix_is_still_rewritten(self):
        """Excluding one package must not turn the run into a no-op, and `demo_pkg` must not
        take `demo_pkg_extra` with it - athena_driver_launch has three such siblings."""
        before = self.manifest("demo_pkg_extra").read_bytes()

        self.validator(excluded_packages=["demo_pkg"]).check_and_format(
            [str(self.tmpdir)]
        )

        self.assertNotEqual(before, self.manifest("demo_pkg_extra").read_bytes())

    def test_it_matches_the_name_tag_not_the_directory(self):
        """A repository routinely has a `foo/` directory holding a package called something
        else - hector's `ec_swift_launch/` ships `ec_swift_software_launch`."""
        _write_package_xml(self.tmpdir / "some_dir" / "package.xml", "actual_name")
        before = (self.tmpdir / "some_dir" / "package.xml").read_bytes()

        self.validator(excluded_packages=["some_dir"]).check_and_format(
            [str(self.tmpdir)]
        )
        self.assertNotEqual(
            before, (self.tmpdir / "some_dir" / "package.xml").read_bytes()
        )

        _write_package_xml(self.tmpdir / "some_dir" / "package.xml", "actual_name")
        before = (self.tmpdir / "some_dir" / "package.xml").read_bytes()

        self.validator(excluded_packages=["actual_name"]).check_and_format(
            [str(self.tmpdir)]
        )
        self.assertEqual(
            before, (self.tmpdir / "some_dir" / "package.xml").read_bytes()
        )

    def test_an_excluded_package_cannot_fail_the_run(self):
        """The whole point: a package nobody wants checked must not hold the exit code."""
        broken = self.tmpdir / "broken" / "package.xml"
        broken.parent.mkdir()
        broken.write_text(
            '<?xml version="1.0"?>\n<package format="3"><name>broken</name></package>\n',
            encoding="utf-8",
        )

        failing = self.validator(check_only=True).check_and_format([str(broken.parent)])
        passing = self.validator(
            check_only=True, excluded_packages=["broken"]
        ).check_and_format([str(broken.parent)])

        self.assertFalse(failing, "the fixture is supposed to be invalid")
        self.assertTrue(passing)

    def test_the_skip_is_announced(self):
        """A package that quietly stops being checked stays unchecked."""
        with self.assertLogs("package_xml_validation.package_xml_validator") as logs:
            self.validator(excluded_packages=["demo_pkg"]).check_and_format(
                [str(self.tmpdir)]
            )

        self.assertTrue(
            any("Skipping demo_pkg (--exclude-package)" in one for one in logs.output),
            f"no skip line in {logs.output}",
        )

    def test_without_the_option_every_package_is_processed(self):
        with self.assertLogs("package_xml_validation.package_xml_validator") as logs:
            self.validator().check_and_format([str(self.tmpdir)])

        self.assertTrue(any("Processing demo_pkg." in one for one in logs.output))
        self.assertTrue(any("Processing demo_pkg_extra." in one for one in logs.output))
        self.assertFalse(any("(--exclude-package)" in one for one in logs.output))
