#!/usr/bin/env python3
import os
import sys
import subprocess
import importlib.util
import builtins
import argparse
import urllib.request
import tempfile
import shutil

# --- GitRepo Class Definition (Compatible with LiteX) ---

class GitRepo:
    def __init__(self, url, clone="regular", develop=True, editable=True, sha1=None, branch="master", tag=None):
        """
        GitRepo definition.
        :param tag: Can be a bool (True means 'taggable' via global --tag) or a str (a specific default tag).
        """
        assert clone in ["regular", "recursive"]
        self.url      = url
        self.clone    = clone
        self.develop  = develop
        self.editable = editable
        self.sha1     = sha1
        self.branch   = branch
        self.tag      = tag

# Inject into builtins so config files can use it without import
builtins.GitRepo = GitRepo

# --- Helper Functions ---

def run_command(cmd, cwd=None, exit_on_error=True):
    print(f"Running: {' '.join(cmd)} (in {cwd or '.'})")
    try:
        subprocess.check_call(cmd, cwd=cwd)
    except subprocess.CalledProcessError as e:
        if exit_on_error:
            print(f"Error: Command failed with exit code {e.returncode}")
            sys.exit(1)
        return False
    return True

def get_config(config_file):
    if not os.path.exists(config_file):
        return None
    try:
        spec = importlib.util.spec_from_file_location("config", config_file)
        config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config)
        return config
    except Exception as e:
        print(f"Error loading config {config_file}: {e}")
        sys.exit(1)

def get_repo_url(name, repo):
    url = repo.url
    if url.startswith("git@") or url.endswith(".git"):
        return url
    if not url.endswith("/"):
        url += "/"
    return url + name + ".git"

# --- Core Actions ---

def do_install(config, deps_dir, editable_override=None, bypass_config=False, tag_override=None):
    os.makedirs(deps_dir, exist_ok=True)
    
    if not config:
        print("Error: No config provided. Action requires a valid configuration file.")
        sys.exit(1)

    git_repos = config.git_repos
    
    if tag_override:
        print(f"Tag override: Using tag '{tag_override}' for taggable repositories (from config).")
    elif bypass_config:
        print("Bypassing config SHA1s/tags. Updating repositories to latest branches (from config)...")

    for name, repo in git_repos.items():
        repo_path = os.path.join(deps_dir, name)
        print(f"\n--- [{name}] ---")
        
        # 1. Clone
        if not os.path.exists(repo_path):
            url = get_repo_url(name, repo)
            print(f"Cloning {url}...")
            # Use --recursive if requested in the definition
            clone_cmd = ["git", "clone"]
            if repo.clone == "recursive":
                clone_cmd.append("--recursive")
            clone_cmd.extend([url, name])
            run_command(clone_cmd, cwd=deps_dir)
        else:
            print("Repository already exists.")

        # 2. Fetch and Checkout
        # Priority 1: Tag override (if repo is taggable)
        # We use the tag provided via CLI (--tag) if the repo definition has tag=True or tag="some_default"
        if tag_override and (repo.tag is not None):
            print(f"Checking out Tag (Override): {tag_override}")
            run_command(["git", "fetch", "--tags"], cwd=repo_path)
            try:
                # Try to get SHA1 of the tag first
                tag_sha1 = subprocess.check_output(["git", "rev-list", "-n", "1", tag_override], cwd=repo_path, stderr=subprocess.DEVNULL).decode().strip()
                run_command(["git", "checkout", "-q", tag_sha1], cwd=repo_path)
            except subprocess.CalledProcessError:
                # Fallback to direct checkout
                run_command(["git", "checkout", "-q", tag_override], cwd=repo_path)
        
        # Priority 2: Fixed SHA1 (from config or repo definition)
        # We don't use fixed SHA1 if --new is used (bypass_config)
        elif repo.sha1 is not None and not bypass_config:
            sha1 = repo.sha1
            if isinstance(sha1, int):
                sha1 = f"{sha1:07x}"
            print(f"Checking out SHA1: {sha1}")
            run_command(["git", "fetch"], cwd=repo_path)
            run_command(["git", "checkout", "-q", sha1], cwd=repo_path)
            
        # Priority 3: Default Tag (if defined in repo and not bypass_config)
        elif repo.tag is not None and not isinstance(repo.tag, bool) and not bypass_config:
            print(f"Checking out Tag (Default): {repo.tag}")
            run_command(["git", "fetch", "--tags"], cwd=repo_path)
            try:
                tag_sha1 = subprocess.check_output(["git", "rev-list", "-n", "1", repo.tag], cwd=repo_path, stderr=subprocess.DEVNULL).decode().strip()
                run_command(["git", "checkout", "-q", tag_sha1], cwd=repo_path)
            except subprocess.CalledProcessError:
                run_command(["git", "checkout", "-q", repo.tag], cwd=repo_path)
                
        # Priority 4: Branch (default or latest)
        else:
            branch = repo.branch or "master"
            if bypass_config:
                print(f"Updating to latest (bypass config)...")
                run_command(["git", "fetch"], cwd=repo_path)
                # Try to find default branch if bypass_config
                try:
                    remote_info = subprocess.check_output(["git", "remote", "show", "origin"], cwd=repo_path, stderr=subprocess.DEVNULL).decode()
                    for line in remote_info.splitlines():
                        if "HEAD branch" in line:
                            branch = line.split(":")[-1].strip()
                            break
                except:
                    pass
            
            print(f"Checking out Branch: {branch}")
            run_command(["git", "checkout", "-q", branch], cwd=repo_path)
            # Only pull if it's a branch and not a fixed commit or tag
            if not (repo.sha1 or (repo.tag and not isinstance(repo.tag, bool))):
                run_command(["git", "pull", "--ff-only"], cwd=repo_path)

        # 3. Submodules
        if repo.clone == "recursive":
            print("Updating submodules...")
            run_command(["git", "submodule", "update", "--init", "--recursive"], cwd=repo_path)

        # 4. Pip Install
        if repo.develop:
            print("Installing via pip...")
            is_editable = repo.editable
            if editable_override is not None:
                is_editable = editable_override
            
            install_cmd = [sys.executable, "-m", "pip", "install"]
            if is_editable:
                install_cmd.append("-e")
            install_cmd.append(".")
            run_command(install_cmd, cwd=repo_path)

def do_check(config, deps_dir):
    if not config:
        print("Error: Config required for check.")
        return False
        
    all_ok = True
    print(f"{'Repository':30} | {'Status':15} | {'Details'}")
    print("-" * 60)
    
    for name, repo in config.git_repos.items():
        repo_path = os.path.join(deps_dir, name)
        if not os.path.exists(repo_path):
            print(f"{name:30} | MISSING         |")
            all_ok = False
            continue
            
        try:
            current_sha1 = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo_path).decode().strip()
            
            if repo.sha1 is not None:
                target_sha1 = repo.sha1
                if isinstance(target_sha1, int):
                    target_sha1 = f"{target_sha1:07x}"
                if current_sha1.startswith(target_sha1):
                    print(f"{name:30} | OK              | SHA1: {current_sha1[:7]}")
                else:
                    print(f"{name:30} | MISMATCH        | Current: {current_sha1[:7]}, Expected: {target_sha1}")
                    all_ok = False
            elif repo.tag is not None:
                try:
                    tag_sha1 = subprocess.check_output(["git", "rev-list", "-n", "1", repo.tag], cwd=repo_path, stderr=subprocess.DEVNULL).decode().strip()
                    if current_sha1 == tag_sha1:
                        print(f"{name:30} | OK              | Tag: {repo.tag}")
                    else:
                        print(f"{name:30} | MISMATCH        | Current: {current_sha1[:7]}, Tag: {repo.tag} ({tag_sha1[:7]})")
                        all_ok = False
                except:
                    print(f"{name:30} | ERROR           | Tag {repo.tag} not found")
                    all_ok = False
            else:
                current_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=repo_path).decode().strip()
                target_branch = repo.branch or "master"
                if current_branch == target_branch:
                    print(f"{name:30} | OK              | Branch: {current_branch}")
                else:
                    print(f"{name:30} | BRANCH MISMATCH | Current: {current_branch}, Expected: {target_branch}")
        except Exception as e:
            print(f"{name:30} | ERROR           | {e}")
            all_ok = False
            
    return all_ok

def do_freeze(deps_dir, config_file, current_config_repos=None):
    print(f"Freezing versions from {deps_dir} to {config_file}...")
    if not os.path.exists(deps_dir):
        print(f"Error: {deps_dir} does not exist.")
        return
        
    repos = {}
    for d in sorted(os.listdir(deps_dir)):
        path = os.path.join(deps_dir, d)
        if os.path.isdir(path) and os.path.exists(os.path.join(path, ".git")):
            try:
                url = subprocess.check_output(["git", "remote", "get-url", "origin"], cwd=path).decode().strip()
                # Clean up URL to base form if possible
                base_url = url
                if base_url.endswith(".git"): base_url = base_url[:-4]
                if base_url.endswith("/" + d): base_url = base_url[:-(len(d)+1)]
                if not base_url.endswith("/"): base_url += "/"
                
                sha1 = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=path).decode().strip()
                branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=path).decode().strip()
                if branch == "HEAD":
                    # Try to find if a branch points here
                    try:
                        branches = subprocess.check_output(["git", "branch", "--contains", "HEAD"], cwd=path).decode()
                        for line in branches.splitlines():
                            b = line.strip().replace("* ", "")
                            if b != "(detached from " and "(" not in b:
                                branch = b
                                break
                    except: pass
                
                has_submodules = os.path.exists(os.path.join(path, ".gitmodules"))
                
                repos[d] = {
                    "url": base_url,
                    "clone": "recursive" if has_submodules else "regular",
                    "sha1": f"0x{sha1[:7]}",
                    "branch": branch
                }
            except:
                print(f"Skipping {d}: not a standard git repo or no remote.")

    with open(config_file, "w") as f:
        f.write("git_repos = {\n")
        for name, r in repos.items():
            f.write(f'    "{name}": GitRepo(\n')
            f.write(f'        url     = "{r["url"]}",\n')
            f.write(f'        clone   = "{r["clone"]}",\n')
            f.write(f'        develop = True,\n')
            f.write(f'        sha1    = {r["sha1"]},\n')
            f.write(f'        branch  = "{r["branch"]}",\n')
            # If repo was taggable in current session, keep it taggable in frozen config
            tag_val = "None"
            if current_config_repos and name in current_config_repos:
                orig_repo = current_config_repos[name]
                if orig_repo.tag is not None:
                    tag_val = str(orig_repo.tag)
            f.write(f'        tag     = {tag_val},\n')
            f.write(f'    ),\n')
        f.write("}\n")
    print("Freeze complete.")

# --- Main Entry Point ---

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LiteX Environment Manager (Native)")
    parser.add_argument("command", choices=["install", "check", "freeze"])
    parser.add_argument("--config", default="litex_config.py", help="Path to config file")
    parser.add_argument("--deps-dir", default="deps", help="Directory for dependencies")
    parser.add_argument("--editable", action="store_true", default=None, help="Force editable install")
    parser.add_argument("--non-editable", action="store_false", dest="editable", help="Force non-editable install")
    parser.add_argument("--new", action="store_true", help="Ignore SHA1s/tags in config and install latest branches")
    parser.add_argument("--tag", help="Use this tag for all taggable repositories defined in config")
    
    args = parser.parse_args()
    
    config = get_config(args.config)
    if config is None:
        print(f"Error: Config file {args.config} not found. A configuration file is now required.")
        sys.exit(1)
            
    if args.command == "install":
        do_install(config, args.deps_dir, editable_override=args.editable, bypass_config=args.new, tag_override=args.tag)
    elif args.command == "check":
        if not do_check(config, args.deps_dir):
            sys.exit(1)
    elif args.command == "freeze":
        do_freeze(args.deps_dir, args.config, current_config_repos=(config.git_repos if config else None))
