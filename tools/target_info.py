import sys
import os
import json
import argparse
import importlib.util

# Store the original class
OriginalArgumentParser = argparse.ArgumentParser

# A simple class that will be used to replace ArgumentParser
class SpyParser:
    def __init__(self, *args, **kwargs):
        self._actions = []
        self._action_groups = []
        self._defaults = {}
        self.description = kwargs.get('description', '')

    def add_argument(self, *args, **kwargs):
        # Create a simple mock action
        action = argparse.Action(args, kwargs.get('dest') or args[0].lstrip('-').replace('-', '_'), 
                                 help=kwargs.get('help'), 
                                 default=kwargs.get('default'),
                                 choices=kwargs.get('choices'))
        # Store type-like info
        action.type = kwargs.get('type')
        action.option_strings = [a for a in args if a.startswith('-')]
        
        # Determine if it's a bool
        action._is_bool = kwargs.get('action') in ('store_true', 'store_false')
        
        self._actions.append(action)
        return action

    def add_argument_group(self, *args, **kwargs):
        group = SpyParser(*args, **kwargs)
        group.title = kwargs.get('title', args[0] if args else "Arguments")
        self._action_groups.append(group)
        # Link actions added to group back to main parser for simplicity in this mock
        group.add_argument = lambda *a, **k: self.add_argument(*a, **k)
        return group
    
    def add_mutually_exclusive_group(self, **kwargs):
        return self.add_argument_group(title="Mutually Exclusive")

    def set_defaults(self, **kwargs):
        self._defaults.update(kwargs)

    def parse_args(self, args=None, namespace=None):
        # Instead of parsing, we emit the JSON and exit
        actions_data = []
        for action in self._actions:
            # Extract group name
            group_name = "Arguments"
            for group in self._action_groups:
                # In our mock, we didn't track which action belongs to which group properly,
                # but we can improve it if needed. For now, let's just use the metadata we have.
                pass

            # Identify type
            action_type = "string"
            if getattr(action, '_is_bool', False):
                action_type = "bool"
            elif action.type == int:
                action_type = "int"
            elif action.type == float:
                action_type = "float"

            actions_data.append({
                "dest": action.dest,
                "flags": action.option_strings,
                "help": action.help,
                "default": self._defaults.get(action.dest, action.default),
                "choices": action.choices,
                "type": action_type,
                "group": "Target options" # Default for LiteX
            })
        
        print(json.dumps(actions_data, indent=2))
        sys.exit(0)

    def __getattr__(self, name):
        # Return a mock for any other method
        return lambda *args, **kwargs: None

# Replace the standard ArgumentParser with our Spy
argparse.ArgumentParser = SpyParser

# Also try to patch LiteXArgumentParser if it's already imported or available
try:
    # Use a more careful approach to avoid circular imports during CPU discovery
    import litex.build.parser
    if hasattr(litex.build.parser, 'LiteXArgumentParser'):
        OldLiteXParser = litex.build.parser.LiteXArgumentParser
        class LiteXSpyParser(OldLiteXParser, SpyParser):
            def parse_args(self, args=None, namespace=None):
                return SpyParser.parse_args(self, args, namespace)
        litex.build.parser.LiteXArgumentParser = LiteXSpyParser
except Exception:
    pass

def main():
    if len(sys.argv) < 2:
        print("Usage: target_info.py <target_script_path>")
        sys.exit(1)

    target_path = sys.argv[1]
    if not os.path.exists(target_path):
        print(f"Error: {target_path} not found")
        sys.exit(1)

    # Setup environment
    sys.path.insert(0, os.getcwd())
    
    # Avoid the script actually doing anything besides defining main and calling it
    # We mock sys.argv to be just the script name so parse_args() is called without issues
    sys.argv = [target_path]

    try:
        # Load the module
        spec = importlib.util.spec_from_file_location("target_module", target_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        # Call main if it exists
        if hasattr(module, "main"):
            module.main()
        else:
            print(f"Error: No main() function found in {target_path}")
            sys.exit(1)
    except SystemExit:
        # This is expected when parse_args() is called
        pass
    except Exception as e:
        print(f"Error during extraction: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
