import os
import sys
import importlib
import argparse
from unittest.mock import patch, MagicMock
import json
import inspect
from collections import defaultdict

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, PROJECT_ROOT)

class ProjectAuditor:
    def __init__(self):
        self.results = {}
        self.all_hdl_files = self._find_all_hdl()
        self.all_py_files = self._find_all_py()
        self.used_hdl = defaultdict(lambda: {'targets': set(), 'callers': set()})
        self.used_py = defaultdict(lambda: {'targets': set(), 'callers': set()})

    def _find_all_hdl(self):
        hdl_files = set()
        for root, _, files in os.walk(PROJECT_ROOT):
            rel_root = os.path.relpath(root, PROJECT_ROOT)
            if any(p in rel_root.split(os.sep) for p in ['build', '__pycache__', '.git', '.venv', 'deps']):
                continue
            for f in files:
                if f.endswith(('.v', '.vhd', '.sv')):
                    hdl_files.add(os.path.relpath(os.path.join(root, f), PROJECT_ROOT))
        return hdl_files

    def _find_all_py(self):
        py_files = set()
        for root, _, files in os.walk(PROJECT_ROOT):
            rel_root = os.path.relpath(root, PROJECT_ROOT)
            if any(p in rel_root.split(os.sep) for p in ['build', '__pycache__', '.git', '.venv', 'deps']):
                continue
            for f in files:
                if f.endswith('.py'):
                    py_files.add(os.path.relpath(os.path.join(root, f), PROJECT_ROOT))
        return py_files

    def _get_caller(self, skip_modules=None):
        skip_modules = skip_modules or []
        stack = inspect.stack()
        for frame_info in stack[1:]:
            module = inspect.getmodule(frame_info.frame)
            if module and hasattr(module, '__file__') and module.__file__:
                fpath = os.path.abspath(module.__file__)
                if fpath.startswith(PROJECT_ROOT):
                    rel = os.path.relpath(fpath, PROJECT_ROOT)
                    if any(p in rel.split(os.sep) for p in ['build', 'deps']):
                        continue
                    if rel in skip_modules:
                        continue
                    if not rel.startswith('boards/targets/') and 'project_auditor.py' not in rel:
                        return rel
        return 'top-level'

    def audit_target(self, target_name):
        print(f'Auditing target: {target_name}...')
        module_path = f'boards.targets.{target_name}'
        captured_soc = []
        hdl_calls = []
        py_calls = []

        # Custom import hook to track Python callers
        if isinstance(__builtins__, dict):
            original_import = __builtins__['__import__']
        else:
            original_import = __builtins__.__import__

        def tracked_import(name, globals=None, locals=None, fromlist=(), level=0):
            module = original_import(name, globals, locals, fromlist, level)
            if module and hasattr(module, '__file__') and module.__file__:
                fpath = os.path.abspath(module.__file__)
                if fpath.startswith(PROJECT_ROOT):
                    rel_imported = os.path.relpath(fpath, PROJECT_ROOT)
                    if rel_imported.endswith('.py'):
                        if any(p in rel_imported.split(os.sep) for p in ['build', 'deps']):
                            return module
                        # Determine who is importing this
                        caller = self._get_caller(skip_modules=[rel_imported])
                        py_calls.append((rel_imported, caller))
            return module

        def mock_build(builder_self, *args, **kwargs):
            if not hasattr(builder_self.soc, 'build_name'):
                builder_self.soc.build_name = target_name
            captured_soc.append(builder_self.soc)
            return None
        def mock_add_source(platform_self, path, library=None, copy=False):
            caller = self._get_caller()
            hdl_calls.append((path, caller))
            if not hasattr(platform_self, 'sources'): platform_self.sources = []
            platform_self.sources.append((path, library, copy))
        def mock_vhd2v(platform, instance=None, files=[], **kwargs):
            caller = self._get_caller()
            if caller == 'gateware/common.py':
                # Try to find the caller of add_vhd2v_converter
                stack = inspect.stack()
                for frame_info in stack[1:]:
                    module = inspect.getmodule(frame_info.frame)
                    if module and hasattr(module, '__file__') and module.__file__:
                        fpath = os.path.abspath(module.__file__)
                        if fpath.startswith(PROJECT_ROOT):
                            rel = os.path.relpath(fpath, PROJECT_ROOT)
                            if rel != 'gateware/common.py' and not rel.startswith('boards/targets/') and 'project_auditor.py' not in rel:
                                caller = rel
                                break
            for f in files:
                hdl_calls.append((f, caller))
            return MagicMock()

        def clean_path(path):
            if os.path.isabs(path):
                try:
                    path = os.path.relpath(path, PROJECT_ROOT)
                except ValueError:
                    pass
            path = os.path.normpath(path)
            if path.startswith('./'):
                path = path[2:]
            return path

        test_args = ['--no-soc-json']
        with patch('sys.argv', ['target_script.py'] + test_args), \
             patch('litex.soc.integration.builder.Builder.build', side_effect=mock_build, autospec=True), \
             patch('litex.build.vhd2v_converter.VHD2VConverter', side_effect=mock_vhd2v), \
             patch('litex.build.generic_platform.GenericPlatform.add_source', side_effect=mock_add_source, autospec=True), \
             patch('os.system', return_value=0), \
             patch('subprocess.check_call', return_value=0), \
             patch('subprocess.run', return_value=MagicMock(returncode=0)), \
             patch('builtins.__import__', side_effect=tracked_import):
            try:
                to_delete = [m for m in sys.modules if m.startswith(('gateware', 'boards'))]
                for m in to_delete: del sys.modules[m]
                target_module = importlib.import_module(module_path)
                if hasattr(target_module, 'main'): target_module.main()
            except Exception as e: print(f'  Error: {e}')
        if not captured_soc: return
        soc = captured_soc[0]
        target_hdl = set()
        all_sources = []
        if hasattr(soc.platform, 'sources'): 
            for s in soc.platform.sources: all_sources.append(s[0])
        if hasattr(soc.platform, 'verilog_sources'): 
            for s in soc.platform.verilog_sources: all_sources.append(s[0])
        
        # Merge hdl_calls into target sources for this target
        for path, caller in hdl_calls:
            all_sources.append(path)

        for path in all_sources:
            rel = clean_path(path)
            if any(p in rel.split(os.sep) for p in ['build', 'deps']):
                continue
            target_hdl.add(rel)
            self.used_hdl[rel]['targets'].add(target_name)
        
        for path, caller in hdl_calls:
            rel = clean_path(path)
            if any(p in rel.split(os.sep) for p in ['build', 'deps']):
                continue
            self.used_hdl[rel]['callers'].add(caller)
        
        # Track Python callers
        for rel_imported, caller in py_calls:
            if any(p in rel_imported.split(os.sep) for p in ['build', 'deps']):
                continue
            self.used_py[rel_imported]['callers'].add(caller)

        target_py = set()
        for name, mod in list(sys.modules.items()):
            if hasattr(mod, '__file__') and mod.__file__:
                fpath = os.path.abspath(mod.__file__)
                if fpath.startswith(PROJECT_ROOT):
                    rel_path = os.path.relpath(fpath, PROJECT_ROOT)
                    if rel_path.endswith('.py'):
                        if any(p in rel_path.split(os.sep) for p in ['build', 'deps']):
                            continue
                        target_py.add(rel_path)
                        self.used_py[rel_path]['targets'].add(target_name)
        self.results[target_name] = {'hdl': sorted(list(target_hdl)), 'py': sorted(list(target_py))}

    def run_all(self):
        targets_dir = os.path.join(PROJECT_ROOT, 'boards', 'targets')
        targets = sorted([f[:-3] for f in os.listdir(targets_dir) if f.endswith('.py') and f != '__init__.py'])
        for t in targets: self.audit_target(t)
        self.report()

    def report(self):
        unused_hdl = sorted(list(self.all_hdl_files - set(self.used_hdl.keys())))
        unused_py = sorted(list(self.all_py_files - set(self.used_py.keys())))
        sorted_hdl_usage = {k: {'targets': sorted(list(v['targets'])), 'callers': sorted(list(v['callers']))} for k, v in sorted(self.used_hdl.items())}
        sorted_py_usage = {k: {'targets': sorted(list(v['targets'])), 'callers': sorted(list(v['callers']))} for k, v in sorted(self.used_py.items())}
        
        master_hdl = []
        for path in sorted(list(self.all_hdl_files)):
            usage = self.used_hdl.get(path, {'targets': set(), 'callers': set()})
            master_hdl.append({
                'path': path,
                'used': path in self.used_hdl,
                'targets': sorted(list(usage['targets'])),
                'callers': sorted(list(usage['callers']))
            })
            
        master_py = []
        for path in sorted(list(self.all_py_files)):
            usage = self.used_py.get(path, {'targets': set(), 'callers': set()})
            master_py.append({
                'path': path,
                'used': path in self.used_py,
                'targets': sorted(list(usage['targets'])),
                'callers': sorted(list(usage['callers']))
            })
        
        summary = {
            'targets': dict(sorted(self.results.items())), 
            'unused_hdl': unused_hdl, 
            'unused_py': unused_py, 
            'hdl_usage': sorted_hdl_usage, 
            'py_usage': sorted_py_usage,
            'master_hdl': master_hdl,
            'master_py': master_py
        }
        with open('audit_results.json', 'w') as f: json.dump(summary, f, indent=4)
        print('Done. Results in audit_results.json')

if __name__ == '__main__':
    auditor = ProjectAuditor()
    auditor.run_all()
