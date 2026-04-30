#!/usr/bin/env python3
import os
import sys
import subprocess
import importlib.util
import builtins
import argparse

# --- GitRepo Class Definition (Compatible with LiteX) ---

class GitRepo:
    def __init__(self, url, clone="regular", develop=True, editable=True, sha1=None, branch="master", tag=None):
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

# --- Default LiteX Repositories (Official List Fallback) ---

DEFAULT_GIT_REPOS = {
    # HDL.
    "migen":    GitRepo(url="https://github.com/m-labs/", clone="recursive", editable=False),
    # LiteX SoC builder.
    "pythondata-software-picolibc":    GitRepo(url="https://github.com/litex-hub/", clone="recursive", tag=True),
    "pythondata-software-compiler_rt": GitRepo(url="https://github.com/litex-hub/", tag=True),
    "litex":                           GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    # LiteX Cores Ecosystem.
    "liteiclink":    GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "liteeth":       GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "litedram":      GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "litepcie":      GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "litesata":      GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "litesdcard":    GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "litescope":     GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "litejesd204b":  GitRepo(url="https://github.com/enjoy-digital/", tag=True),
    "litespi":       GitRepo(url="https://github.com/litex-hub/", tag=True),
    "litei2c":       GitRepo(url="https://github.com/litex-hub/", branch="main", tag=True),
    "litex-boards":  GitRepo(url="https://github.com/litex-hub/", tag=True),
    # LiteX Python Data.
    "pythondata-misc-tapcfg":    GitRepo(url="https://github.com/litex-hub/", tag=True),
    "pythondata-misc-usb_ohci":  GitRepo(url="https://github.com/litex-hub/", clone="recursive", tag=True),
    "pythondata-cpu-lm32":       GitRepo(url="https://github.com/litex-hub/", tag=True),
    "pythondata-cpu-mor1kx":     GitRepo(url="https://github.com/litex-hub/", tag=True),
    "pythondata-cpu-minerva":    GitRepo(url="https://github.com/litex-hub/", tag=True),
    "pythondata-cpu-naxriscv":   GitRepo(url="https://github.com/litex-hub/", tag=True),
    "pythondata-cpu-sentinel":   GitRepo(url="https://github.com/litex-hub/", branch="main", tag=True),
    "pythondata-cpu-serv":       GitRepo(url="https://github.com/litex-hub/", tag=True),
    "pythondata-cpu-vexiiriscv": GitRepo(url="https://github.com/litex-hub/", branch="main", tag=True),
    "pythondata-cpu-vexriscv":   GitRepo(url="https://github.com/litex-hub/", tag=True),
    "pythondata-cpu-vexriscv-smp": GitRepo(url="https://github.com/litex-hub/", clone="recursive", tag=True),
}

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
    if not (url.endswith(".git") or url.startswith("git@")):
        if url.endswith("/"):
            url = url + name + ".git"
        else:
            url = url + "/" + name + ".git"
    return url

# --- Core Actions ---

def do_install(config, deps_dir, editable_override=None, bypass_config=False, tag_override=None):
    os.makedirs(deps_dir, exist_ok=True)
    
    if tag_override:
        print(f"Tag override: Using tag '{tag_override}' for all repositories.")
        if bypass_config:
            git_repos = DEFAULT_GIT_REPOS
        else:
            # If we have a config, use repos from config, otherwise fallback to default
            git_repos = config.git_repos if config else DEFAULT_GIT_REPOS
    elif bypass_config:
        print("Bypassing config. Installing newest repositories from official list...")
        git_repos = DEFAULT_GIT_REPOS
    elif config is None:
        print("Error: No config provided and --new/--tag not used.")
        sys.exit(1)
    else:
        git_repos = config.git_repos

    for name, repo in git_repos.items():
        repo_path = os.path.join(deps_dir, name)
        print(f"\n--- [{name}] ---")
        
        # 1. Clone
        if not os.path.exists(repo_path):
            url = get_repo_url(name, repo)
            print(f"Cloning {url}...")
            run_command(["git", "clone", url, name], cwd=deps_dir)
        else:
            print("Repository already exists.")

        # 2. Fetch and Checkout
        if tag_override and (repo.tag is not None):
            print(f"Checking out Tag: {tag_override}")
            run_command(["git", "fetch", "--tags"], cwd=repo_path)
            try:
                # Get the SHA1 of the tag and checkout that SHA1 (LiteX style)
                tag_sha1 = subprocess.check_output(["git", "rev-list", "-n", "1", tag_override], cwd=repo_path, stderr=subprocess.DEVNULL).decode().strip()
                run_command(["git", "checkout", tag_sha1], cwd=repo_path)
            except subprocess.CalledProcessError:
                # Fallback to direct checkout if rev-list fails (maybe it's not a tag but a branch/sha1)
                run_command(["git", "checkout", tag_override], cwd=repo_path)
        elif bypass_config:
            # For --new, we want latest of default branch
            print("Updating to latest...")
            run_command(["git", "fetch"], cwd=repo_path)
            # Find default branch
            try:
                remote_info = subprocess.check_output(["git", "remote", "show", "origin"], cwd=repo_path).decode()
                default_branch = "master"
                for line in remote_info.splitlines():
                    if "HEAD branch" in line:
                        default_branch = line.split(":")[-1].strip()
                        break
                run_command(["git", "checkout", default_branch], cwd=repo_path)
                run_command(["git", "pull", "--ff-only"], cwd=repo_path)
            except:
                run_command(["git", "pull", "--ff-only"], cwd=repo_path)
        else:
            if repo.sha1 is not None:
                sha1 = repo.sha1
                if isinstance(sha1, int):
                    sha1 = f"{sha1:07x}"
                print(f"Checking out SHA1: {sha1}")
                run_command(["git", "fetch"], cwd=repo_path)
                run_command(["git", "checkout", sha1], cwd=repo_path)
            elif repo.tag is not None:
                print(f"Checking out Tag: {repo.tag}")
                run_command(["git", "fetch", "--tags"], cwd=repo_path)
                try:
                    tag_sha1 = subprocess.check_output(["git", "rev-list", "-n", "1", repo.tag], cwd=repo_path, stderr=subprocess.DEVNULL).decode().strip()
                    run_command(["git", "checkout", tag_sha1], cwd=repo_path)
                except subprocess.CalledProcessError:
                    run_command(["git", "checkout", repo.tag], cwd=repo_path)
            else:
                branch = repo.branch or "master"
                print(f"Checking out Branch: {branch}")
                run_command(["git", "checkout", branch], cwd=repo_path)
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

def do_freeze(deps_dir, config_file):
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
            f.write(f'    "{name}" : GitRepo(url="{r["url"]}",\n')
            f.write(f'        clone   = "{r["clone"]}",\n')
            f.write(f'        develop = True,\n')
            f.write(f'        sha1    = {r["sha1"]},\n')
            f.write(f'        branch  = "{r["branch"]}"\n')
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
    parser.add_argument("--new", action="store_true", help="Bypass config and install latest versions")
    parser.add_argument("--tag", help="Override repository version with this tag")
    
    args = parser.parse_args()
    
    config = None
    if not (args.new or args.tag):
        config = get_config(args.config)
        if config is None and args.command in ["check"]:
            print(f"Error: Config file {args.config} not found.")
            sys.exit(1)
            
    if args.command == "install":
        do_install(config, args.deps_dir, editable_override=args.editable, bypass_config=args.new, tag_override=args.tag)
    elif args.command == "check":
        if not do_check(config, args.deps_dir):
            sys.exit(1)
    elif args.command == "freeze":
        do_freeze(args.deps_dir, args.config)
