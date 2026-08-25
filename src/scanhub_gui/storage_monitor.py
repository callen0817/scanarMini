#!/usr/bin/env python3
"""
scanHUB Storage Monitor
Continuously checks disk usage for the scan destination directory.
Enforces the 10% safety auto-stop threshold to prevent dataset corruption.
"""

import shutil
import os

class StorageMonitor:
    def __init__(self, target_path="/home/scanar"):
        self.target_path = os.path.expanduser(target_path)
        
    def get_storage_stats(self):
        """
        Returns a dictionary containing storage statistics:
        - total_gb: Total disk space in GB
        - used_gb: Used disk space in GB
        - free_gb: Available disk space in GB
        - percent_free: Percentage of free disk space (0.0 to 100.0)
        - percent_used: Percentage of used disk space (0.0 to 100.0)
        - is_critical: True if percent_free <= 10.0%
        """
        try:
            total, used, free = shutil.disk_usage(self.target_path)
            total_gb = total / (1024 ** 3)
            used_gb = used / (1024 ** 3)
            free_gb = free / (1024 ** 3)
            percent_free = (free / total) * 100.0 if total > 0 else 0.0
            percent_used = (used / total) * 100.0 if total > 0 else 0.0
            is_critical = percent_free <= 10.0
            
            return {
                "total_gb": round(total_gb, 2),
                "used_gb": round(used_gb, 2),
                "free_gb": round(free_gb, 2),
                "percent_free": round(percent_free, 1),
                "percent_used": round(percent_used, 1),
                "is_critical": is_critical
            }
        except Exception as e:
            return {
                "total_gb": 0.0,
                "used_gb": 0.0,
                "free_gb": 0.0,
                "percent_free": 0.0,
                "percent_used": 100.0,
                "is_critical": True,
                "error": str(e)
            }
