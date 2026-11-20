"""Validation and node ownership for the physical motor simulator."""
import os
import re


def parse_ids(value):
    if not isinstance(value, str):
        raise ValueError('ids must be a comma-separated string')
    result = set()
    for token in value.split(','):
        token = token.strip()
        if not token:
            continue
        match = re.fullmatch(r'(\d+)(?:-(\d+))?', token)
        if not match:
            raise ValueError(f'invalid node IDs: {token}')
        first = int(match[1]); last = int(match[2] or first)
        if not 1 <= first <= last <= 127:
            raise ValueError('node IDs must be in 1..127')
        values = set(range(first, last + 1))
        if result & values:
            raise ValueError('duplicate node IDs')
        result.update(values)
    return result


def validate_routes(rows):
    if not isinstance(rows, list) or not 1 <= len(rows) <= 4:
        raise ValueError('channels must contain 1..4 routes')
    routes = []; buses = set(); endpoints = set(); assigned = set(); remainder = False
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError('each channel must be an object')
        bus = row.get('bus')
        if type(bus) is not int or not 0 <= bus <= 3 or bus in buses:
            raise ValueError('duplicate or invalid MCU bus (expected 0..3)')
        buses.add(bus)
        kind = row.get('transport')
        route = {'bus': bus, 'transport': kind, 'ids': row.get('ids', '')}
        if kind == 'usbcanfd':
            device, channel = row.get('device', 0), row.get('channel')
            if type(device) is not int or not 0 <= device < 8:
                raise ValueError('USB-CAN FD device must be in 0..7')
            if type(channel) is not int or channel not in (0, 1):
                raise ValueError('USB-CAN FD channel must be 0 or 1')
            route.update(device=device, channel=channel)
            endpoint = (kind, device, channel)
        elif kind in ('socketcan', 'slcan'):
            value = row.get('endpoint')
            if not isinstance(value, str) or not value:
                raise ValueError('channel endpoint is required')
            if kind == 'socketcan' and not re.fullmatch(r'[\w.-]{1,15}', value, re.ASCII):
                raise ValueError('invalid SocketCAN endpoint')
            route['endpoint'] = value
            endpoint = (kind, os.path.realpath(value) if kind == 'slcan' else value)
        else:
            raise ValueError('channel transport must be usbcanfd, socketcan or slcan')
        if endpoint in endpoints:
            raise ValueError('duplicate physical endpoint')
        endpoints.add(endpoint)
        if route['ids'] == '*':
            if remainder:
                raise ValueError('only one remainder (*) channel is allowed')
            remainder = True
        else:
            ids = parse_ids(route['ids'])
            if ids & assigned:
                raise ValueError('duplicate node ownership across channels')
            assigned.update(ids)
        routes.append(route)
    return sorted(routes, key=lambda r: r['bus'])


def assign_nodes(rows, node_ids):
    """Select model nodes per route; omitted nodes stay unloaded unless '*' selects them."""
    routes = validate_routes(rows)
    known = set(node_ids)
    if len(known) != len(node_ids):
        raise ValueError('duplicate model IDs across buses/protocols')
    explicit = set().union(*(parse_ids(r['ids']) for r in routes if r['ids'] != '*'))
    if explicit - known:
        raise ValueError(f'unknown model node IDs: {sorted(explicit - known)}')
    for route in routes:
        route['node_ids'] = sorted(known - explicit if route['ids'] == '*' else parse_ids(route['ids']))
    return routes


def enumerate_usbcanfd_devices(root='/sys/bus/usb/devices'):
    from pathlib import Path
    devices = []
    for path in Path(root).glob('*'):
        try:
            if ((path / 'idVendor').read_text().strip(),
                    (path / 'idProduct').read_text().strip()) != ('04d8', '0053'):
                continue
            bus = int((path / 'busnum').read_text())
            address = int((path / 'devnum').read_text())
        except (OSError, ValueError):
            continue
        node = f'/dev/bus/usb/{bus:03d}/{address:03d}'
        devices.append({'bus': bus, 'address': address, 'path': node,
                        'accessible': os.access(node, os.R_OK | os.W_OK)})
    devices.sort(key=lambda d: (d['bus'], d['address']))
    for index, device in enumerate(devices):
        device['index'] = index
        device['label'] = f"创芯 USB-CAN FD #{index + 1} ({device['bus']:03d}/{device['address']:03d})"
    return devices


def validate_endpoints(rows, can_interfaces, serial_ports, usb_devices):
    routes = validate_routes(rows)
    cans = {item['name']: item for item in can_interfaces}
    ports = {item['path']: item for item in serial_ports}
    devices = {item['index']: item for item in usb_devices}
    for route in routes:
        if route['transport'] == 'usbcanfd':
            device = devices.get(route['device'])
            if device is None:
                raise ValueError('select an enumerated USB-CAN FD device')
            if not device.get('accessible'):
                raise PermissionError('bench_manager cannot read and write USB-CAN FD device')
        elif route['transport'] == 'socketcan':
            if route['endpoint'] not in cans:
                raise ValueError('select an enumerated CAN interface')
            if not cans[route['endpoint']].get('up'):
                raise ValueError('SocketCAN interface is down')
        else:
            if route['endpoint'] not in ports:
                raise ValueError('select an enumerated serial device')
            if not os.access(route['endpoint'], os.R_OK | os.W_OK):
                raise PermissionError('bench_manager cannot read and write serial endpoint')
    return routes


def encode_route_config(config):
    """Keep JSON out of systemctl's whitespace-delimited Environment output."""
    import base64
    import json
    result = dict(config)
    for key in ('channels', 'sim_channels_json'):
        if key in result:
            payload = json.dumps(result.pop(key), separators=(',', ':')).encode()
            result[key + '_b64'] = base64.urlsafe_b64encode(payload).decode()
    return result


def decode_route_config(config):
    import base64
    import json
    if config is None:
        return None
    result = dict(config)
    for key in ('channels', 'sim_channels_json'):
        if key + '_b64' in result:
            result[key] = json.loads(base64.urlsafe_b64decode(result.pop(key + '_b64')))
    return result
