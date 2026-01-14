#!/usr/bin/env python3
"""
update_config.py - Safely update YAML configuration files

This script provides safe YAML manipulation for parameter sweeps,
avoiding the whitespace and nesting issues with sed-based updates.

Usage:
    python3 update_config.py --config configs/experiment_config.yaml --set cpu_load.cpu_percent=90
    python3 update_config.py --config configs/experiment_config.yaml --set load_scenarios.cpu_load.cpu_percent=90
    python3 update_config.py --config configs/experiment_config.yaml --get cpu_load.cpu_percent
    python3 update_config.py --config configs/experiment_config.yaml --backup  # Create timestamped backup
    python3 update_config.py --config configs/experiment_config.yaml --restore backup_file.yaml

Features:
    - Preserves YAML formatting and comments (when using ruamel.yaml)
    - Handles nested parameters with dot notation
    - Atomic writes (write to temp, then rename)
    - Backup/restore functionality
    - Validation of parameter paths

Requirements:
    pip install pyyaml
    # Optional for comment preservation: pip install ruamel.yaml
"""

import argparse
import os
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

# Try to use ruamel.yaml for better formatting preservation
try:
    from ruamel.yaml import YAML
    HAS_RUAMEL = True
except ImportError:
    HAS_RUAMEL = False


def load_config(config_path: Path) -> Tuple[Dict, Any]:
    """
    Load YAML configuration file.

    Returns:
        Tuple of (config_dict, yaml_handler)
        yaml_handler is used for saving with formatting preservation
    """
    if HAS_RUAMEL:
        yaml_handler = YAML()
        yaml_handler.preserve_quotes = True
        with open(config_path) as f:
            config = yaml_handler.load(f)
        return dict(config) if config else {}, yaml_handler
    else:
        with open(config_path) as f:
            config = yaml.safe_load(f)
        return config or {}, None


def save_config(config: Dict, config_path: Path, yaml_handler: Any = None):
    """
    Save configuration to file atomically.

    Uses write-to-temp-then-rename pattern to prevent corruption.
    """
    # Create temp file in same directory (for atomic rename)
    config_dir = config_path.parent
    fd, temp_path = tempfile.mkstemp(
        suffix='.yaml',
        prefix='.config_',
        dir=config_dir
    )

    try:
        with os.fdopen(fd, 'w') as f:
            if yaml_handler and HAS_RUAMEL:
                yaml_handler.dump(config, f)
            else:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)

        # Atomic rename
        os.rename(temp_path, config_path)
    except Exception as e:
        # Clean up temp file on failure
        if os.path.exists(temp_path):
            os.unlink(temp_path)
        raise e


def get_nested_value(config: Dict, key_path: str) -> Tuple[Any, bool]:
    """
    Get value from nested config using dot notation.

    Args:
        config: Configuration dictionary
        key_path: Dot-separated path like "load_scenarios.cpu_load.cpu_percent"

    Returns:
        Tuple of (value, found)
    """
    keys = key_path.split('.')
    current = config

    for key in keys:
        if isinstance(current, dict) and key in current:
            current = current[key]
        else:
            # Try to find in common nested locations
            found = False
            for section in ['load_scenarios', 'benchmark', 'tracing', 'metrics']:
                if section in config and isinstance(config[section], dict):
                    if key in config[section]:
                        current = config[section][key]
                        found = True
                        break
                    # Check one level deeper
                    for sub_key, sub_val in config[section].items():
                        if isinstance(sub_val, dict) and key in sub_val:
                            current = sub_val[key]
                            found = True
                            break
                    if found:
                        break

            if not found:
                return None, False

    return current, True


def set_nested_value(config: Dict, key_path: str, value: Any) -> bool:
    """
    Set value in nested config using dot notation.

    Args:
        config: Configuration dictionary
        key_path: Dot-separated path like "load_scenarios.cpu_load.cpu_percent"
        value: Value to set (will be type-converted)

    Returns:
        True if value was set, False if path not found
    """
    keys = key_path.split('.')

    # Try direct path first
    current = config
    for key in keys[:-1]:
        if isinstance(current, dict) and key in current:
            current = current[key]
        else:
            current = None
            break

    # Set value if path exists
    final_key = keys[-1]
    if isinstance(current, dict) and final_key in current:
        current[final_key] = convert_value(value)
        return True

    # Try to find in common nested locations
    search_locations = [
        ('load_scenarios', 'cpu_load'),
        ('load_scenarios', 'msg_load'),
        ('benchmark', 'planning'),
        ('benchmark', 'timeouts'),
        ('benchmark', 'goal_pose'),
        ('tracing',),
        ('metrics',),
    ]

    for location in search_locations:
        target = config
        valid = True
        for loc_key in location:
            if isinstance(target, dict) and loc_key in target:
                target = target[loc_key]
            else:
                valid = False
                break

        if valid and isinstance(target, dict):
            # Check if final key exists in this location
            if final_key in target:
                target[final_key] = convert_value(value)
                return True
            # Also check original key path in case it's partially specified
            if len(keys) > 1 and keys[-2] in config and isinstance(config[keys[-2]], dict):
                if final_key in config[keys[-2]]:
                    config[keys[-2]][final_key] = convert_value(value)
                    return True

    return False


def convert_value(value_str: str) -> Any:
    """Convert string value to appropriate type."""
    # Try int
    try:
        return int(value_str)
    except ValueError:
        pass

    # Try float
    try:
        return float(value_str)
    except ValueError:
        pass

    # Try bool
    if value_str.lower() in ('true', 'yes', 'on'):
        return True
    if value_str.lower() in ('false', 'no', 'off'):
        return False

    # Return as string
    return value_str


def create_backup(config_path: Path) -> Path:
    """Create timestamped backup of config file."""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    backup_path = config_path.parent / f"{config_path.stem}_backup_{timestamp}{config_path.suffix}"
    shutil.copy2(config_path, backup_path)
    return backup_path


def restore_backup(config_path: Path, backup_path: Path):
    """Restore config from backup file."""
    if not backup_path.exists():
        raise FileNotFoundError(f"Backup file not found: {backup_path}")
    shutil.copy2(backup_path, config_path)


def list_parameters(config: Dict, prefix: str = "") -> List[str]:
    """List all parameters in config with dot notation paths."""
    params = []
    for key, value in config.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            params.extend(list_parameters(value, path))
        else:
            params.append(f"{path}={value}")
    return params


def main():
    parser = argparse.ArgumentParser(
        description='Safely update YAML configuration files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Set a parameter value
  python3 update_config.py --config configs/experiment_config.yaml --set cpu_percent=90

  # Set nested parameter
  python3 update_config.py --config configs/experiment_config.yaml --set load_scenarios.cpu_load.cpu_percent=90

  # Get a parameter value
  python3 update_config.py --config configs/experiment_config.yaml --get cpu_percent

  # Create backup before modifying
  python3 update_config.py --config configs/experiment_config.yaml --backup

  # List all parameters
  python3 update_config.py --config configs/experiment_config.yaml --list
        """
    )

    parser.add_argument('--config', '-c', required=True,
                        help='Path to YAML config file')
    parser.add_argument('--set', '-s', metavar='KEY=VALUE',
                        help='Set parameter value (dot notation for nested)')
    parser.add_argument('--get', '-g', metavar='KEY',
                        help='Get parameter value')
    parser.add_argument('--backup', '-b', action='store_true',
                        help='Create timestamped backup')
    parser.add_argument('--restore', '-r', metavar='BACKUP_FILE',
                        help='Restore from backup file')
    parser.add_argument('--list', '-l', action='store_true',
                        help='List all parameters')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Suppress output except errors')

    args = parser.parse_args()

    config_path = Path(args.config)

    if not config_path.exists():
        print(f"ERROR: Config file not found: {config_path}", file=sys.stderr)
        return 1

    # Handle restore
    if args.restore:
        try:
            restore_backup(config_path, Path(args.restore))
            if not args.quiet:
                print(f"Restored config from: {args.restore}")
            return 0
        except Exception as e:
            print(f"ERROR: Failed to restore: {e}", file=sys.stderr)
            return 1

    # Handle backup
    if args.backup:
        try:
            backup_path = create_backup(config_path)
            if not args.quiet:
                print(f"Backup created: {backup_path}")
            return 0
        except Exception as e:
            print(f"ERROR: Failed to create backup: {e}", file=sys.stderr)
            return 1

    # Load config
    try:
        config, yaml_handler = load_config(config_path)
    except Exception as e:
        print(f"ERROR: Failed to load config: {e}", file=sys.stderr)
        return 1

    # Handle list
    if args.list:
        params = list_parameters(config)
        for param in sorted(params):
            print(param)
        return 0

    # Handle get
    if args.get:
        value, found = get_nested_value(config, args.get)
        if found:
            print(value)
            return 0
        else:
            print(f"ERROR: Parameter not found: {args.get}", file=sys.stderr)
            return 1

    # Handle set
    if args.set:
        if '=' not in args.set:
            print(f"ERROR: Invalid format. Use KEY=VALUE", file=sys.stderr)
            return 1

        key, value = args.set.split('=', 1)

        if set_nested_value(config, key, value):
            try:
                save_config(config, config_path, yaml_handler)
                if not args.quiet:
                    print(f"Set {key}={value}")
                return 0
            except Exception as e:
                print(f"ERROR: Failed to save config: {e}", file=sys.stderr)
                return 1
        else:
            print(f"ERROR: Parameter not found: {key}", file=sys.stderr)
            return 1

    # No action specified
    parser.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
