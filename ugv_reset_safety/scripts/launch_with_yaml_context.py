#!/usr/bin/env python3
"""Materialize runtime pose/topic context into the YAML passed to roslaunch."""
import json
import math
import os
from pathlib import Path
import re
import sys
import uuid

import yaml

CONTEXT_FIELDS = {
    'reset_initial_x', 'reset_initial_y', 'reset_initial_yaw',
    'cmd_vel_topic', 'scene_namespace',
    'fence_x_min', 'fence_x_max', 'fence_y_min', 'fence_y_max',
}
FENCE_YAML_KEYS = {
    'fence_x_min': 'x_min',
    'fence_x_max': 'x_max',
    'fence_y_min': 'y_min',
    'fence_y_max': 'y_max',
}


def prepare_parameters(source_text, context, namespace):
    from roslaunch.substitution_args import resolve_args

    if not isinstance(context, dict) or set(context) - CONTEXT_FIELDS:
        raise ValueError('only Experiment pose and topic context may be materialized')
    values = yaml.safe_load(resolve_args(source_text, context={'arg': {'ns': namespace}}))
    if not isinstance(values, dict):
        raise ValueError('configuration must be a YAML mapping')
    for key, value in context.items():
        if key.startswith('reset_initial_'):
            if value == '':
                continue
            if isinstance(value, bool):
                raise ValueError('reset pose must contain finite numbers')
            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ValueError('reset pose must contain finite numbers')
            if not math.isfinite(value):
                raise ValueError('reset pose must contain finite numbers')
            values[key] = value
        elif key in FENCE_YAML_KEYS:
            if value == '':
                continue
            if isinstance(value, bool):
                raise ValueError('fence must contain finite numbers')
            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ValueError('fence must contain finite numbers')
            if not math.isfinite(value):
                raise ValueError('fence must contain finite numbers')
            fence = values.get('fence')
            if not isinstance(fence, dict):
                fence = {}
            fence[FENCE_YAML_KEYS[key]] = value
            values['fence'] = fence
        elif not isinstance(value, str) or not re.fullmatch(r'/?[A-Za-z_][A-Za-z0-9_/]*', value):
            raise ValueError('invalid ROS topic context: ' + key)
        else:
            values[key] = value
    return values


def main(argv):
    if len(argv) != 5:
        print('usage: launch_with_yaml_context.py PACKAGE LAUNCH NAMESPACE CONFIG_FILE CONTEXT_JSON', file=sys.stderr)
        return 2
    package, launch, namespace, config, raw_context = argv
    try:
        parameters = prepare_parameters(Path(config).read_text(), json.loads(raw_context), namespace)
        directory = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros'))) / 'xgc-controller-configurations' / str(uuid.uuid4())
        directory.mkdir(parents=True)
        loaded = directory / 'parameters.yaml'
        loaded.write_text(yaml.safe_dump(parameters))
        (directory / 'manifest.yaml').write_text(yaml.safe_dump({'source': config, 'context': json.loads(raw_context), 'loaded_file': str(loaded), 'parameters': parameters}))
    except (OSError, ValueError) as error:
        print('controller YAML: ' + str(error), file=sys.stderr)
        return 2
    command = ['roslaunch', '--wait', package, launch, 'config_file:=' + str(loaded)]
    if namespace:
        command.append('ns:=' + namespace)
    print('Controller configuration: ' + str(loaded), file=sys.stderr, flush=True)
    if os.environ.get('XGC_PRINT_LAUNCH_ARGS') == '1':
        print(json.dumps(command))
        return 0
    os.execvp(command[0], command)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
