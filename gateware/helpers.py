# Helper utilities for gateware-related tasks.
#
# This module provides utilities to explore a Migen/LiteX module hierarchy and
# export it as a JSON tree. It also provides helpers to record object
# instantiation call-sites for better traceability in the exported hierarchy.

import inspect
import json


# --- Public API -----------------------------------------------------------------

def module_to_tree(mod, name=None, visited=None, path=None):
    """Return a dict representing the Module/LiteXModule hierarchy.

    Parameters
    - mod:        Root module (Migen Module or LiteXModule).
    - name:       Optional attribute name used to reference the module from parent.
    - visited:    Set of visited object ids to avoid cycles.
    - path:       Full path from the SoC root to this node (computed internally when None).

    The function tries to map submodules to attribute names by inspecting
    attributes whose value is a Module/LiteXModule and that are present in
    mod.submodules. It also looks into lists/tuples/dicts of such modules.
    """
    if visited is None:
        visited = set()
    node_id = id(mod)

    # Compute this node's path
    this_name = name or mod.__class__.__name__
    this_path = this_name if (path is None) else path

    if node_id in visited:
        # Even in cycles, try to include definition/instantiation info when available
        def_file, def_line = None, None
        try:
            def_file = inspect.getsourcefile(mod.__class__) or inspect.getfile(mod.__class__)
            def_line = inspect.getsourcelines(mod.__class__)[1]
        except Exception:
            pass
        return {
            "name": this_name,
            "type": mod.__class__.__name__,
            "path": this_path,
            "cycle": True,
            "def_file": def_file,
            "def_line": def_line,
            "inst_file": getattr(mod, "_inst_file", None),
            "inst_line": getattr(mod, "_inst_line", None),
            "children": []
        }
    visited.add(node_id)

    # Try to find class definition file/line
    def_file, def_line = None, None
    try:
        def_file = inspect.getsourcefile(mod.__class__) or inspect.getfile(mod.__class__)
        def_line = inspect.getsourcelines(mod.__class__)[1]
    except Exception:
        pass

    # Gather the set/list of submodules managed by migen
    try:
        sublist = list(getattr(mod, "submodules", []))
    except Exception:
        sublist = []

    # Map object -> names discovered via attributes
    child_entries = []

    def add_child(child_obj, child_name):
        # Import locally to avoid heavy imports at module import time if not needed
        try:
            from migen import Module as _M  # type: ignore
            is_module = isinstance(child_obj, _M)
        except Exception:
            is_module = False
        if not is_module:
            return
        # Only include if part of mod.submodules when available; if sublist empty, include anyway
        if sublist and (child_obj not in sublist):
            return
        child_entries.append((child_name, child_obj))

    # Inspect attributes
    for attr, val in list(vars(mod).items()):
        # Skip private/internal or huge non-module references
        if attr.startswith("_"):
            continue
        # Direct module attribute
        add_child(val, attr)
        # Containers
        if isinstance(val, (list, tuple)):
            for i, item in enumerate(val):
                add_child(item, f"{attr}[{i}]")
        elif isinstance(val, dict):
            for k, item in val.items():
                add_child(item, f"{attr}[{k}]")

    # Also include unnamed submodules that were added but not found via attributes
    named_objs = {obj for (_, obj) in child_entries}
    for obj in sublist:
        if obj not in named_objs:
            child_entries.append((obj.__class__.__name__, obj))

    # Build children trees, deduplicating by object id and name
    seen_ids = set()
    children = []
    for child_name, child_obj in child_entries:
        cid = id(child_obj)
        if cid in seen_ids:
            continue
        seen_ids.add(cid)
        child_path = f"{this_path}.{child_name}" if this_path else child_name
        children.append(module_to_tree(child_obj, child_name, visited, path=child_path))

    return {
        "name": this_name,
        "type": mod.__class__.__name__,
        "path": this_path,
        "def_file": def_file,
        "def_line": def_line,
        "inst_file": getattr(mod, "_inst_file", None),
        "inst_line": getattr(mod, "_inst_line", None),
        "children": children,
    }


# --- Post-processing helpers to infer instantiation locations from parent source files.

def _guess_inst_location(_src_path, _child_name):
    """Best-effort: scan parent source file for a likely instantiation assignment of child_name.

    Returns (file, line) on success, (None, None) otherwise.
    """
    try:
        import re
        with open(_src_path, "r") as _f:
            lines = _f.readlines()
    except Exception:
        return None, None

    # Common patterns: direct assignment, submodules assignment, add_module(name="child")
    patterns = [
        # self.child = Something(...)
        (lambda name: re.compile(rf"^\s*self\.{re.escape(name)}\s*=\s*.+")),
        # self.submodules.child = Something(...)
        (lambda name: re.compile(rf"^\s*self\.submodules\.{re.escape(name)}\s*=\s*.+")),
        # self.add_module(name="child", ...)
        (lambda name: re.compile(rf"add_module\s*\(\s*name\s*=\s*[\"\']{re.escape(name)}[\"\']")),
    ]

    for i, line in enumerate(lines, start=1):
        for patf in patterns:
            if patf(_child_name).search(line):
                return _src_path, i
    return None, None


def _fill_missing_inst_info(_node, _parent_def_file=None):
    """Recursively fill missing inst_file/inst_line using the parent's def_file and child's name."""
    try:
        # Only try to infer when missing and we know the parent source file
        if _node.get("inst_file") is None and _parent_def_file and _node.get("name"):
            inst_f, inst_l = _guess_inst_location(_parent_def_file, _node["name"])
            if inst_f:
                _node["inst_file"] = inst_f
                _node["inst_line"] = inst_l

        # Recurse to children, using this node's def_file as their parent source file
        for _child in _node.get("children", []) or []:
            _fill_missing_inst_info(_child, _node.get("def_file"))
    except Exception:
        # Be resilient: never let best-effort inference break JSON generation
        pass


def write_module_hierarchy_json(mod, outfile="soc_structure.json", name="SoC"):
    """Build hierarchy tree of `mod`, infer missing inst locations, and write to JSON file."""
    tree = module_to_tree(mod, name=name)
    _fill_missing_inst_info(tree, None)
    js = json.dumps(tree, indent=2)
    with open(outfile, "w") as f:
        f.write(js + "\n")
