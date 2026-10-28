"""Temporary BVH/URDF conversion jobs; browser preview only, no ROS publishers."""

import copy
import json
from pathlib import Path
import tempfile
import threading
from urllib.parse import urlsplit
import uuid
import xml.etree.ElementTree as ET


class MotionJobs:
    def __init__(self):
        self.directory = tempfile.TemporaryDirectory(prefix='workbench-motion-')
        self.jobs = {}
        self.lock = threading.RLock()
        self.stopping = threading.Event()
        self.thread = None

    def _job(self, key):
        if key not in self.jobs:
            raise FileNotFoundError('动作任务已过期，请重新导入')
        return self.jobs[key]

    def status(self, key):
        with self.lock:
            return copy.deepcopy(self._job(key)['status'])

    def create(self, request):
        from bvh_broadcaster.bvh import BvhClip
        from ament_index_python.packages import get_package_share_directory
        import numpy as np
        import pinocchio as pin
        import yaml
        if not isinstance(request, dict):
            raise ValueError('请提交 BVH 和 URDF 文件')
        for field in ('bvh_text', 'urdf_text'):
            if not isinstance(request.get(field), str) or not request[field].strip():
                raise ValueError('BVH 和 URDF 均不能为空')
        up_axis = request.get('up_axis', 'y_up')
        scale = request.get('scale', .01)
        if up_axis not in ('y_up', 'z_up') or scale not in (.001, .01, 1.):
            raise ValueError('不支持的 BVH 坐标或单位')
        if '<!DOCTYPE' in request['urdf_text'].upper() or '<!ENTITY' in request['urdf_text'].upper():
            raise ValueError('URDF 不支持 DTD 或实体声明')
        robot = ET.fromstring(request['urdf_text'])
        if robot.tag != 'robot':
            raise ValueError('请选择有效的 URDF robot 文件')
        links = [link.attrib['name'] for link in robot.findall('link')]
        edges = [(j.find('parent').attrib['link'], j.find('child').attrib['link'])
                 for j in robot.findall('joint')]
        children = [c for _, c in edges]
        roots = set(links) - set(children)
        if (len(set(links)) != len(links) or len(roots) != 1 or len(set(children)) != len(children)
                or any(p not in links or c not in links for p, c in edges)):
            raise ValueError('URDF 必须是连杆名称唯一、单根的完整关节树')
        seen, pending = set(), list(roots)
        while pending:
            parent = pending.pop()
            if parent in seen:
                raise ValueError('URDF 存在循环关节')
            seen.add(parent)
            pending.extend(c for p, c in edges if p == parent)
        if seen != set(links):
            raise ValueError('URDF 包含断开的连杆或循环关节')
        with self.lock:
            if any(j['status']['state'] == 'running' for j in self.jobs.values()):
                raise ValueError('已有转换正在进行，请等待完成')
            while len(self.jobs) >= 3:
                old = next(iter(self.jobs))
                import shutil
                shutil.rmtree(self.jobs.pop(old)['directory'])
            key = uuid.uuid4().hex
            directory = Path(self.directory.name) / key
            directory.mkdir()
            try:
                (directory / 'source.bvh').write_text(request['bvh_text'], encoding='utf-8')
                (directory / 'robot.urdf').write_text(request['urdf_text'], encoding='utf-8')
                clip = BvhClip.load(directory / 'source.bvh')
                model = pin.buildModelFromUrdf(str(directory / 'robot.urdf'))
                if (model.nq < 1 or any(j.nq != 1 or j.nv != 1 for j in list(model.joints)[1:])
                        or not np.isfinite([model.lowerPositionLimit, model.upperPositionLimit,
                                           model.velocityLimit]).all()
                        or np.any(model.lowerPositionLimit >= model.upperPositionLimit)
                        or np.any(model.velocityLimit <= 0)):
                    raise ValueError('当前流程支持有位置、速度限位的独立双臂单自由度关节')
                sources = [j.name for j in clip.joints if not j.name.endswith('__end')]
                template = yaml.safe_load((Path(get_package_share_directory('bvh_broadcaster')) /
                                           'config/half_body_robot.yaml').read_text())
                automatic = (all(name in sources for arm in template['arms'].values()
                                 for name in arm['source']) and template['torso'] in sources
                             and all(name in links for arm in template['arms'].values()
                                     for name in arm['robot']))
                status = {'id': key, 'state': 'mapping', 'progress': 0,
                          'bvh_name': str(request.get('bvh_name', 'source.bvh'))[:160],
                          'urdf_name': str(request.get('urdf_name', 'robot.urdf'))[:160],
                          'robot_name': robot.attrib.get('name', 'robot'),
                          'frames': len(clip.frames), 'frame_time': clip.frame_time,
                          'joint_count': model.nq, 'source_joints': sources, 'robot_links': links,
                          'automatic_mapping': automatic, 'mapping': template if automatic else None}
                self.jobs[key] = {'directory': directory, 'clip': clip, 'scale': scale,
                                  'up_axis': up_axis, 'status': status}
            except Exception:
                import shutil
                shutil.rmtree(directory)
                raise
        if automatic:
            return self.start(key, template)
        return self.status(key)

    def start(self, key, mapping):
        from bvh_broadcaster.retarget import ArmRetargeter
        with self.lock:
            job = self._job(key)
            if job['status']['state'] not in ('mapping', 'failed'):
                raise ValueError('该任务已开始，请勿重复提交')
            if any(j['status']['state'] == 'running' for j in self.jobs.values()):
                raise ValueError('已有转换正在进行，请等待完成')
            if not isinstance(mapping, dict):
                raise ValueError('请先选择人体与机器人关节对应关系')
            solver = ArmRetargeter(job['directory'] / 'robot.urdf', mapping, job['clip'],
                                   up_axis=job['up_axis'])
            job['status'].update(state='running', progress=0, error='', mapping=copy.deepcopy(mapping))
            self.thread = threading.Thread(target=self._convert, args=(key, solver), daemon=True)
            self.thread.start()
            return copy.deepcopy(job['status'])

    def _convert(self, key, solver):
        import numpy as np
        import pinocchio as pin
        from scipy.spatial.transform import Rotation
        from bvh_broadcaster.retarget import digest, load_motion
        job = self.jobs[key]
        directory, clip = job['directory'], job['clip']

        def progress(frame, total):
            if self.stopping.is_set():
                raise RuntimeError('服务已停止，请重新导入')
            with self.lock:
                job['status']['progress'] = round(frame / total * 90)
        try:
            positions, directions, orientations = solver.convert(progress)
            np.savez_compressed(directory / 'motion.npz', positions=positions,
                                joint_names=solver.names, frame_time=clip.frame_time,
                                urdf_sha256=digest(directory / 'robot.urdf'),
                                direction_error_deg=directions, orientation_error_deg=orientations)
            load_motion(directory / 'motion.npz', directory / 'robot.urdf')
            links = job['status']['robot_links']
            link_ids = [solver.model.getFrameId(n, pin.FrameType.BODY) for n in links]
            children = {j.find('child').attrib['link']: j.find('parent').attrib['link']
                        for j in ET.parse(directory / 'robot.urdf').getroot().findall('joint')}
            robot_parents = [links.index(children[n]) if n in children else -1 for n in links]
            # Downsample display geometry only; the downloaded joint motion keeps every source frame.
            indices = sorted(set(range(0, len(positions), max(1, (len(positions) + 899) // 900)))
                             | {len(positions) - 1})
            source_frames, robot_frames = [], []
            for index in indices:
                if self.stopping.is_set():
                    raise RuntimeError('服务已停止')
                ps, rs = [], []
                for joint, (p, quat) in zip(clip.joints, clip.local_transforms(
                        index, scale=job['scale'], up_axis=job['up_axis'], apply_root_offset=False)):
                    r = Rotation.from_quat(quat).as_matrix()
                    if joint.parent >= 0:
                        p, r = ps[joint.parent] + rs[joint.parent] @ p, rs[joint.parent] @ r
                    ps.append(p)
                    rs.append(r)
                source_frames.append(np.round(ps, 5).tolist())
                pin.framesForwardKinematics(solver.model, solver.data, positions[index])
                robot_frames.append(np.round([solver.data.oMf[f].translation for f in link_ids], 5).tolist())
            preview = {'times': (np.array(indices) * clip.frame_time).tolist(),
                       'duration': (len(positions) - 1) * clip.frame_time,
                       'source': {'names': [j.name for j in clip.joints],
                                  'parents': [j.parent for j in clip.joints], 'frames': source_frames},
                       'robot': {'names': links, 'parents': robot_parents, 'frames': robot_frames,
                                 'urdf': (directory / 'robot.urdf').read_text(),
                                 'joint_names': solver.names,
                                 'joint_positions': np.round(positions[indices], 8).tolist()},
                       'source_to_robot': job['status']['mapping']['source_to_robot']}
            (directory / 'preview.json').write_text(json.dumps(preview, allow_nan=False))
            summary = {'preview_only': True, 'frames': len(positions), 'joint_names': solver.names,
                       'frame_time': clip.frame_time, 'mapping': job['status']['mapping'],
                       'direction_mean_deg': np.mean(directions, axis=0).tolist(),
                       'hand_orientation_mean_deg': np.mean(orientations, axis=0).tolist(),
                       'nonconverged_solves': solver.nonconverged}
            (directory / 'report.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
            with self.lock:
                job['status'].update(state='done', progress=100, summary=summary)
        except Exception as exc:
            with self.lock:
                job['status'].update(state='failed', error=str(exc))

    def result(self, key, name):
        if name not in ('preview.json', 'motion.npz', 'report.json'):
            raise FileNotFoundError('结果不存在')
        with self.lock:
            job = self._job(key)
            if job['status']['state'] != 'done':
                raise ValueError('转换尚未完成')
            return (job['directory'] / name).read_bytes()

    def close(self):
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=10)
        if not self.thread or not self.thread.is_alive():
            self.directory.cleanup()


def handle_motion(handler, jobs):
    """Shared HTTP adapter for the production server and isolated browser tests."""
    path = urlsplit(handler.path).path
    if not path.startswith('/api/motion/'):
        return False

    def send(status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        handler.send_response(status)
        handler.send_header('Content-Type', 'application/json; charset=utf-8')
        handler.send_header('Content-Length', str(len(body)))
        handler.send_header('Cache-Control', 'no-store')
        handler.end_headers()
        handler.wfile.write(body)

    try:
        parts = path.strip('/').split('/')
        if handler.command == 'POST':
            origin = handler.headers.get('Origin')
            if origin and urlsplit(origin).netloc != handler.headers.get('Host'):
                send(403, {'error': '只接受同源动作请求'})
                return True
            if handler.headers.get_content_type() != 'application/json':
                send(415, {'error': '动作请求必须为 application/json'})
                return True
            length = int(handler.headers.get('Content-Length', '0'))
            if not 0 < length <= 24 * 1024 * 1024:
                send(413, {'error': '导入文件合计不能超过 24 MB'})
                return True
            request = json.loads(handler.rfile.read(length))
            if not isinstance(request, dict):
                raise ValueError('动作请求必须是 JSON 对象')
            if parts == ['api', 'motion', 'jobs']:
                send(202, jobs.create(request))
            elif len(parts) == 5 and parts[:3] == ['api', 'motion', 'jobs'] and parts[4] == 'convert':
                send(202, jobs.start(parts[3], request.get('mapping')))
            else:
                send(404, {'error': '动作接口不存在'})
        elif handler.command == 'GET' and parts[:3] == ['api', 'motion', 'jobs']:
            if len(parts) == 4:
                send(200, jobs.status(parts[3]))
            elif len(parts) == 5:
                body = jobs.result(parts[3], parts[4])
                handler.send_response(200)
                handler.send_header('Content-Type', 'application/octet-stream' if parts[4].endswith('.npz')
                                    else 'application/json; charset=utf-8')
                handler.send_header('Content-Length', str(len(body)))
                handler.send_header('Cache-Control', 'no-store')
                if parts[4] != 'preview.json':
                    handler.send_header('Content-Disposition', f'attachment; filename="{parts[4]}"')
                handler.end_headers()
                handler.wfile.write(body)
            else:
                send(404, {'error': '动作接口不存在'})
        else:
            send(404, {'error': '动作接口不存在'})
    except FileNotFoundError as exc:
        send(404, {'error': str(exc)})
    except ImportError:
        send(503, {'error': '转换组件未就绪，请构建 bvh_broadcaster 并加载 ROS 工作区环境'})
    except Exception as exc:
        send(400, {'error': str(exc)})
    return True
