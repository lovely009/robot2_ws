#!/usr/bin/env python3
"""Generate strict TypeScript declarations from the production FastAPI BFF OpenAPI schema."""
from __future__ import annotations

from pathlib import Path
import sys

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from web_host.bff.app import create_app
from web_host.bff.ros_gateway import MockRosGateway


def _schema_to_ts(schema: dict | None) -> str:
    if not schema:
        return 'unknown'
    if '$ref' in schema:
        return schema['$ref'].rsplit('/', 1)[-1]
    if 'const' in schema:
        v = schema['const']
        return repr(v).replace("'", '"') if isinstance(v, str) else str(v)
    if 'enum' in schema:
        return ' | '.join(
            repr(v).replace("'", '"') if isinstance(v, str)
            else str(v).lower() if isinstance(v, bool) else str(v)
            for v in schema['enum']
        )
    if 'anyOf' in schema:
        parts = [_schema_to_ts(sub) for sub in schema['anyOf']]
        return ' | '.join(dict.fromkeys(parts))
    kind = schema.get('type')
    if kind == 'null':
        return 'null'
    if kind == 'string':
        return 'string'
    if kind in ('integer', 'number'):
        return 'number'
    if kind == 'boolean':
        return 'boolean'
    if kind == 'array':
        item = _schema_to_ts(schema.get('items'))
        return f'({item})[]' if ' | ' in item else f'{item}[]'
    if kind == 'object':
        addl = schema.get('additionalProperties')
        if isinstance(addl, dict):
            return f'Record<string, {_schema_to_ts(addl)}>'
        props = schema.get('properties')
        if isinstance(props, dict):
            req = set(schema.get('required', ()))
            fields = [
                f"{k}{'' if k in req else '?'}: {_schema_to_ts(v)}"
                for k, v in props.items()
            ]
            return '{ ' + '; '.join(fields) + ' }'
        return 'Record<string, unknown>'
    return 'unknown'


def render_typescript(openapi_doc: dict) -> str:
    lines = [
        '/* eslint-disable */',
        '// Auto-generated from FastAPI OpenAPI schema by scripts/generate_bff_types.py.',
        '',
    ]
    schemas = openapi_doc.get('components', {}).get('schemas', {})
    for name in sorted(schemas):
        spec = schemas[name]
        props = spec.get('properties', {})
        req = set(spec.get('required', ()))
        lines.append(f'export interface {name} {{')
        for prop_name, prop_schema in props.items():
            opt = '' if prop_name in req else '?'
            lines.append(f'  {prop_name}{opt}: {_schema_to_ts(prop_schema)};')
        lines.append('}')
        lines.append('')

    lines.append('export interface BffEndpoints {')
    for path, methods in sorted(openapi_doc.get('paths', {}).items()):
        for method, op in sorted(methods.items()):
            req_schema = (
                op.get('requestBody', {})
                .get('content', {})
                .get('application/json', {})
                .get('schema')
            )
            res_obj = op.get('responses', {})
            res_entry = res_obj.get('200') or res_obj.get('202') or {}
            res_schema = (
                res_entry.get('content', {})
                .get('application/json', {})
                .get('schema')
            )
            req_ts = _schema_to_ts(req_schema) if req_schema else 'never'
            res_ts = _schema_to_ts(res_schema) if res_schema else 'unknown'
            lines.append(f'  "{method.upper()} {path}": {{ request: {req_ts}; response: {res_ts} }};')
    lines.append('}')
    lines.append('')
    return '\n'.join(lines)


def main() -> int:
    app = create_app(ros_gateway=MockRosGateway())
    out_path = PKG_ROOT / 'frontend' / 'src' / 'api' / 'bff.d.ts'
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(render_typescript(app.openapi()), encoding='utf-8')
    print(f'Generated {out_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
