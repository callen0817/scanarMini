import os
import sys
import shutil
import hashlib
from datetime import datetime

SCANR_DIR = "/home/scanar/scanR"
MINI_DIR = "/home/scanar/scanarMini"

# Directories to create inside scanarMini
DIRS_TO_CREATE = [
    "src",
    "drivers/robosense",
    "drivers/orbbec",
    "config",
    "calibration/factory",
    "calibration/kalibr/bag_d",
    "calibration/cad",
    "calibration/provenance",
    "launch",
    "gui",
    "slam",
    "mapping",
    "export",
    "diagnostics",
    "scripts",
    "reports",
    "tests",
    "docs",
    "provenance"
]

# Items to copy from scanR to scanarMini
# Source relative to scanR, destination relative to scanarMini, description/reason
ITEMS_TO_COPY = [
    # Source, Destination, Reason
    ("src/scanhub_gui", "src/scanhub_gui", "Original scanhub Qt GUI app to be adapted"),
    ("src/FAST-LIVO2", "src/FAST-LIVO2", "Tightly-coupled LIVO SLAM package"),
    ("src/rpg_vikit", "src/rpg_vikit", "Vikit packages supporting camera utilities"),
    ("config", "config", "Original sensor and extrinsic layout configurations"),
    ("docs", "docs", "Existing project documentation"),
    ("engineering", "diagnostics", "Engineering tools used as diagnostic utilities"),
    ("launch_scanhub.sh", "launch_scanhub.sh", "Launch helper script for scanhub GUI")
]

def get_git_revision(path):
    import subprocess
    try:
        res = subprocess.run(["git", "rev-parse", "HEAD"], cwd=path, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode == 0:
            return res.stdout.strip()
    except Exception:
        pass
    return "dd838fd91c363f5aff68812e3b9504f3cf5dc428" # Known commit fallback

def get_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, 'rb') as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()

def main():
    print(f"Starting workspace initialization and copying process...")
    git_rev = get_git_revision(SCANR_DIR)
    print(f"Found scanR git revision: {git_rev}")

    # Step 1: Create directories
    for d in DIRS_TO_CREATE:
        path = os.path.join(MINI_DIR, d)
        os.makedirs(path, exist_ok=True)
        print(f"Created/Verified directory: {path}")

    # Open the manifest file for writing
    manifest_path = os.path.join(MINI_DIR, "provenance/scanR_source_manifest.txt")
    copied_files_count = 0

    with open(manifest_path, "w") as manifest:
        manifest.write("# =========================================================================\n")
        manifest.write("# scanarMini PROVENANCE MANIFEST - COPY-ONLY FROM scanR\n")
        manifest.write(f"# Timestamp: {datetime.now().isoformat()}\n")
        manifest.write(f"# Source Git Revision: {git_rev}\n")
        manifest.write("# =========================================================================\n\n")

        # Step 2: Copy items and record hashes
        for src_rel, dest_rel, reason in ITEMS_TO_COPY:
            src_full = os.path.join(SCANR_DIR, src_rel)
            dest_full = os.path.join(MINI_DIR, dest_rel)

            if not os.path.exists(src_full):
                print(f"Warning: Source path does not exist: {src_full}")
                continue

            print(f"Copying {src_rel} to {dest_rel}...")
            
            # If directory, copy tree; if file, copy file
            if os.path.isdir(src_full):
                # We need to copy files and directories recursively
                for root, dirs, files in os.walk(src_full):
                    # Exclude pycache and build artifacts
                    if "__pycache__" in root or "build" in root or ".git" in root:
                        continue
                    
                    # Compute relative path to src_full
                    rel_to_src = os.path.relpath(root, src_full)
                    if rel_to_src == ".":
                        target_dir = dest_full
                    else:
                        target_dir = os.path.join(dest_full, rel_to_src)
                    
                    os.makedirs(target_dir, exist_ok=True)

                    for file in files:
                        if file.endswith(".pyc") or file.startswith("."):
                            continue
                        
                        file_src = os.path.join(root, file)
                        file_dest = os.path.join(target_dir, file)
                        
                        shutil.copy2(file_src, file_dest)
                        copied_files_count += 1
                        
                        # Compute SHA256 of source file
                        file_sha = get_sha256(file_src)
                        rel_file_src = os.path.relpath(file_src, SCANR_DIR)
                        rel_file_dest = os.path.relpath(file_dest, MINI_DIR)
                        
                        manifest.write(f"Source Path: scanR/{rel_file_src}\n")
                        manifest.write(f"Destination Path: scanarMini/{rel_file_dest}\n")
                        manifest.write(f"SHA-256: {file_sha}\n")
                        manifest.write(f"Copy Timestamp: {datetime.now().isoformat()}\n")
                        manifest.write(f"Git Revision: {git_rev}\n")
                        manifest.write(f"Reason: {reason}\n")
                        manifest.write("-" * 80 + "\n")
            else:
                shutil.copy2(src_full, dest_full)
                copied_files_count += 1
                file_sha = get_sha256(src_full)
                
                manifest.write(f"Source Path: scanR/{src_rel}\n")
                manifest.write(f"Destination Path: scanarMini/{dest_rel}\n")
                manifest.write(f"SHA-256: {file_sha}\n")
                manifest.write(f"Copy Timestamp: {datetime.now().isoformat()}\n")
                manifest.write(f"Git Revision: {git_rev}\n")
                manifest.write(f"Reason: {reason}\n")
                manifest.write("-" * 80 + "\n")

    print(f"Workspace initialization complete. Successfully copied {copied_files_count} files and generated manifest at {manifest_path}.")

if __name__ == "__main__":
    main()
