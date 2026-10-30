"""Verify the hardware source graph and preprocessor exclude the hosted hook."""
import os
from pathlib import Path
import runpy
import subprocess
import unittest


PROJECT = Path(__file__).resolve().parents[1] / 'MCB-project'


class MappingHookIsolation(unittest.TestCase):
    def test_hardware_source_graph(self):
        class Environment:
            def Clone(self):
                return self

            def AppendUnique(self, **kwargs):
                pass

            def FindSourceFiles(self, directory, ignorePaths, ignoreFiles):
                self.ignored = ignorePaths
                return []

        for target, excluded in [('hardware', True), ('sim', False)]:
            env = Environment()
            runpy.run_path(str(PROJECT / 'src/SConscript'), init_globals={
                'Import': lambda name: None, 'Return': lambda name: None,
                'env': env, 'c_flags': ['SENTRY'], 'sources': [],
                'args': {'TARGET_ENV': target, 'ROBOT_TYPE': 'SENTRY'}})
            self.assertEqual('hosted' in env.ignored, excluded)

    def test_hardware_preprocessor_has_no_control_interface(self):
        result = subprocess.run([
            os.environ.get('CXX', 'g++'), '-E', '-P', '-x', 'c++',
            str(PROJECT / 'src/hosted/mapping_control.hpp')],
            capture_output=True, text=True, check=True)
        self.assertEqual(result.stdout.strip(), '')


if __name__ == '__main__':
    unittest.main(verbosity=2)
