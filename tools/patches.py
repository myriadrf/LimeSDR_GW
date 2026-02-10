#
# This file is part of LimeSDR_GW.
#

import re
import importlib

def get_litex_memory_patch_targets():
    """Finds LiteX and Migen memory generator functions to patch."""
    targets = []
    # Candidate locations for Verilog generation functions
    candidate_locations = [
        ("litex.gen.fhdl.memory", "_memory_generate_verilog"),
        ("litex.gen.fhdl.verilog", "_memory_generate_verilog"),
        ("migen.fhdl.specials",    "Memory"),
    ]

    for module_path, attr_name in candidate_locations:
        try:
            module = importlib.import_module(module_path)
            target_obj = getattr(module, attr_name)

            # Special case for Migen Memory: we patch Memory.emit_verilog
            if attr_name == "Memory":
                targets.append((target_obj, "emit_verilog", target_obj.emit_verilog))
            else:
                targets.append((module, attr_name, target_obj))
        except (ImportError, AttributeError):
            pass

    return targets

class LiteXMemoryPatcher:
    """
    Patches LiteX/Migen memory generators to force specific FPGA RAM styles
    (e.g., 'distributed' or 'block') for selected memories based on heuristics.
    """
    def __init__(self, min_width=300, max_depth=64, name_filter=None, limit=None, ram_style="distributed", verbose=True):
        self.min_width   = min_width
        self.max_depth   = max_depth
        self.name_filter = name_filter
        self.limit       = limit
        self.ram_style   = ram_style
        self.verbose     = verbose

        self.patch_count = 0
        self.patched_memories = set()

    def patch(self, targets=None):
        """Applies the patches to the identified generator functions."""
        applied_names = []
        patch_targets = targets or get_litex_memory_patch_targets()

        for module_or_obj, attr, original_func in patch_targets:
            current_func = getattr(module_or_obj, attr)
            if not hasattr(current_func, "_patched"):
                setattr(module_or_obj, attr, self._wrap(original_func))
                # Friendly identifier for the patched generator
                module_name = getattr(module_or_obj, "__name__", str(module_or_obj)).split('.')[-1]
                applied_names.append(f"{module_name}.{attr}")

        if self.verbose and applied_names:
            print(f"[LiteXMemoryPatcher] Applied patches to {len(applied_names)} memory generators: {', '.join(applied_names)}")
            limit_str = self.limit if self.limit is not None else '∞'
            print(f"[LiteXMemoryPatcher] Configuration: style={self.ram_style}, limit={limit_str}, min_width={self.min_width}")

        return len(applied_names) > 0

    def _wrap(self, original_func):
        """Wraps the original generator to inject Verilog attributes."""
        def wrapper(*args, **kwargs):
            # Detect signature and extract memory object and namespace
            # LiteX: (name, memory, namespace, ...)
            # Migen: (memory, namespace, ...)
            if len(args) > 3:
                memory_obj, namespace = args[1], args[2]
            else:
                memory_obj, namespace = args[0], args[1]

            name = namespace.get_name(memory_obj)
            code = original_func(*args, **kwargs)

            # Use a stable key to track memories across multiple LiteX passes
            memory_key = (memory_obj.width, memory_obj.depth, name)

            # Check if this memory is a candidate for patching
            is_already_patched = memory_key in self.patched_memories

            width_match  = memory_obj.width >= self.min_width
            depth_match  = memory_obj.depth <= self.max_depth
            filter_match = not self.name_filter or any(f in name for f in self.name_filter)
            limit_match  = self.limit is None or self.patch_count < self.limit

            should_patch = is_already_patched or (width_match and depth_match and filter_match and limit_match)

            if should_patch:
                attribute_prefix = f'(* ram_style = "{self.ram_style}" *) '
                if attribute_prefix in code:
                    return code

                # Attempt to inject attribute before 'reg' declaration
                reg_declaration = f"reg [{memory_obj.width-1}:0] {name}["
                if reg_declaration in code:
                    patched_code = code.replace(reg_declaration, attribute_prefix + reg_declaration)
                    injection_success = True
                else:
                    # Fallback to regex for varying whitespace
                    regex_pattern = rf"(reg\s+\[\s*{memory_obj.width-1}\s*:\s*0\s*\]\s+{re.escape(name)}\s*\[)"
                    patched_code, match_count = re.subn(regex_pattern, attribute_prefix + r'\1', code)
                    injection_success = match_count > 0

                if injection_success:
                    if not is_already_patched:
                        self.patch_count += 1
                        self.patched_memories.add(memory_key)

                    if self.verbose:
                        # Log success in green
                        print(f"\033[92m[LiteXMemoryPatcher] Forced {self.ram_style} ram style for {name} ({memory_obj.width}x{memory_obj.depth})\033[0m")
                    return patched_code

            return code

        wrapper._patched = True
        return wrapper
