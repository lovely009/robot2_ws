"""Import a validated body capture into the same saved BVH catalog as the workbench."""

import json
from pathlib import Path

from .actions import write_action_text
from .motion import MotionJobs


IMPORT_PATH = '/api/motion/imports/vdmocap'


class ImportConflict(ValueError):
    """The recording is valid but the current workbench state prevents publication."""


def _check_targets(status, model, builtin):
    if status['mapping_origin'] == 'invalid':
        raise ImportConflict('已有映射需要重新确认，请在动作工作台检查该映射')
    targets = [t for t in status['mapping']['targets'] if t.get('source')]
    if not targets or (status['mapping_origin'] != 'saved' and not builtin and
                       not all(t.get('confidence') == 'preset' for t in targets)):
        raise ValueError(f'{model} 缺少已确认的身体骨架映射或适用的模型预设')
    return len(targets)


class _MappingCheck(MotionJobs):
    """Inspect using the normal solver before allocating one of the three shared jobs."""

    def _generic_mapping(self, job, request, known=None):
        result = super()._generic_mapping(job, request, known)
        self.builtin = known is not None
        return result


def _verify_mapping(jobs, params):
    checker = _MappingCheck(profiles_dir=jobs.profiles_dir, models_root=jobs.models_root)
    try:
        status = checker.create(params)
        _check_targets(status, params['model'], checker.builtin)
        return checker.builtin
    finally:
        checker.close()


def _already_published(path, text):
    if not path.exists():
        return False
    if path.is_file() and path.stat().st_size == len(text) and path.read_bytes() == text.encode('ascii'):
        return True
    raise FileExistsError('动作库已有同名的其他录制，请另取动作名')


def _publish_mapping(jobs, job, mapping, was_saved):
    # Use the workbench mapping format. Only explicit user confirmation may replace a record.
    path = jobs.profiles_dir / (job['profile_key'] + '.json')
    reference = job['clip'].frames[mapping.get('reference_frame', 0)].tolist()
    record = {'version': 1, 'mapping': mapping, 'reference': reference}
    if not was_saved and not path.exists():
        try:
            write_action_text(path, json.dumps(record, allow_nan=False))
            return
        except FileExistsError:
            pass
    try:
        if path.stat().st_size <= 1024 * 1024:
            current = json.loads(path.read_text())
            if isinstance(current, dict) and all(current.get(k) == v for k, v in record.items()):
                return
    except (FileNotFoundError, ValueError):
        pass
    raise ImportConflict('已确认的骨架映射发生变化，请重新提交录制')


def import_capture(jobs, request):
    from .models import _models_root, enumerate_arm_models
    from .vdmocap import body_bvh

    if not isinstance(request, dict):
        raise ValueError('动捕导入请求必须是 JSON 对象')
    name = jobs._source_name(request.get('name'))
    if (name.startswith('.') or not name.lower().endswith('.bvh') or not name.isprintable()
            or len(name.encode()) > 255):
        raise ValueError('动作名称须为单层、可显示的 .bvh 文件名')
    if jobs.actions_dir is None or jobs.cache is None or jobs.profiles_dir is None:
        raise OSError('动作库、映射或结果目录未配置')
    model = request.get('model', 'full_body_robot')
    root = jobs.models_root or _models_root()
    if root is None or model not in enumerate_arm_models(root):
        raise ValueError(f'模型库中找不到目标模型 {model!r}')
    urdf = Path(root) / model / 'urdf' / f'{model}.urdf'
    text, info = body_bvh(request.get('capture'))
    params = {'model': model, 'urdf_text': urdf.read_text(), 'urdf_name': urdf.name,
              'bvh_text': text, 'bvh_name': name, 'scale': .01, 'up_axis': 'auto',
              'retargeting': 'generic', 'review_mapping': True}
    destination = jobs.actions_dir / name
    with jobs.lock:
        if any(j['status']['state'] == 'running' for j in jobs.jobs.values()):
            raise ImportConflict('已有转换正在进行，请等待完成后重试')
        _already_published(destination, text)
        builtin = _verify_mapping(jobs, params)
        status = jobs.create(params)
        job = jobs.jobs[status['id']]
        matched = _check_targets(status, model, builtin)
        mapping, was_saved = status['mapping'], status['mapping_origin'] == 'saved'
        status = jobs.start(status['id'], mapping)
        worker = jobs.thread if status['state'] == 'running' else None

    if worker:
        worker.join()
    with jobs.lock:
        status = job['status']
        if status['state'] != 'done':
            raise ValueError(status.get('error') or '动作解算未完成')
        if not status.get('result_saved'):
            raise OSError(status.get('cache_warning') or '关节解算结果未能保存')
        published = _already_published(destination, text)
        _publish_mapping(jobs, job, mapping, was_saved)
        if not published:
            try:
                write_action_text(destination, text)
            except FileExistsError:
                if not _already_published(destination, text):
                    raise
        return {**info, 'state': 'done', 'name': name, 'model': model, 'job_id': status['id'],
                'matched_targets': matched, 'joint_count': status['joint_count'],
                'preview_only': status['summary']['preview_only'], 'result_cached': bool(status.get('result_cached'))}
