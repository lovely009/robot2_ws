"""Offline, read-only checking and explicit uncommissioned template creation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml

from .validation import (
    ValidationError, compute_fingerprint, create_template, validate_configuration,
)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('check', 'fingerprint', 'template'))
    parser.add_argument('--driver-file', required=True)
    parser.add_argument('--urdf-file', required=True)
    parser.add_argument('--model-name', default='')
    parser.add_argument('--robot-id', default='')
    parser.add_argument('--manifest-file', default='')
    parser.add_argument('--output', default='')
    options = parser.parse_args(args)
    try:
        if options.command == 'fingerprint':
            print(compute_fingerprint(options.driver_file, options.urdf_file))
            return 0
        if options.command == 'check':
            result = validate_configuration(
                options.driver_file, options.urdf_file, options.manifest_file,
                options.model_name, options.robot_id)
            print(json.dumps(result.asdict(), ensure_ascii=False, allow_nan=False))
            return 0 if result.valid else 1
        if not options.output:
            raise ValidationError('template requires --output; an existing file is never overwritten')
        template = create_template(options.driver_file, options.urdf_file,
                                   options.model_name, options.robot_id)
        with Path(options.output).open('x', encoding='utf-8') as stream:
            yaml.safe_dump(template, stream, allow_unicode=True, sort_keys=False)
        return 0
    except (ValidationError, OSError, RecursionError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
