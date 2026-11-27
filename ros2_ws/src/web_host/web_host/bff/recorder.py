"""rosbag2 (MCAP / sqlite3) recorder attached to RosGatewayProtocol raw sink."""
from __future__ import annotations

import json
from pathlib import Path
import re
import shutil
import tempfile
import threading
import time
from typing import Literal
import zipfile

from .ros_gateway import RosGatewayProtocol
from .schemas import RecordingInfo, RecordingStatus

_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,95}$')


def _detect_storage_format() -> Literal['mcap', 'sqlite3']:
    try:
        import rosbag2_py
        with tempfile.TemporaryDirectory() as tmp:
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(Path(tmp) / 'probe'), storage_id='mcap'),
                rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr',
                ),
            )
            del writer
        return 'mcap'
    except Exception:
        return 'sqlite3'


class RosbagRecorder:
    """Manages a single active rosbag2 recording session and saved bag artifacts."""

    def __init__(self, recordings_dir: Path, gateway: RosGatewayProtocol) -> None:
        self._dir = Path(recordings_dir)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._gateway = gateway
        self._storage_format: Literal['mcap', 'sqlite3'] = _detect_storage_format()
        self._lock = threading.Lock()

        self._active = False
        self._name: str | None = None
        self._bag_dir: Path | None = None
        self._writer = None
        self._started_mono = 0.0
        self._started_ms = 0
        self._message_count = 0
        self._topic_filter: set[str] | None = None
        self._registered_topics: set[str] = set()

    @property
    def storage_format(self) -> Literal['mcap', 'sqlite3']:
        return self._storage_format

    def _validate_name(self, name: str) -> str:
        if not name or not _NAME_RE.match(name):
            raise ValueError('录制名称只能包含字母、数字、下划线和连字符')
        return name

    def _safe_bag_dir(self, name: str) -> Path:
        valid = self._validate_name(name)
        root = self._dir.resolve()
        target = (root / valid).resolve()
        if target.parent != root:
            raise ValueError('录制路径非法')
        return target

    def status(self) -> RecordingStatus:
        with self._lock:
            if not self._active:
                return RecordingStatus(active=False, storage_format=self._storage_format)
            return RecordingStatus(
                active=True,
                name=self._name,
                storage_format=self._storage_format,
                started_at_ms=self._started_ms,
                duration_s=round(max(0.0, time.monotonic() - self._started_mono), 3),
                message_count=self._message_count,
                topics=sorted(self._registered_topics),
            )

    def start(self, name: str = '', topics: list[str] | None = None) -> RecordingStatus:
        import rosbag2_py

        clean = name.strip() if name else f'rec_{time.strftime("%Y%m%d_%H%M%S")}'
        bag_dir = self._safe_bag_dir(clean)

        with self._lock:
            if self._active:
                raise RuntimeError('已有录制任务正在进行中')
            if bag_dir.exists():
                raise FileExistsError(f'录制 {clean} 已存在')

            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=self._storage_format),
                rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr',
                ),
            )
            self._active = True
            self._name = clean
            self._bag_dir = bag_dir
            self._writer = writer
            self._started_mono = time.monotonic()
            self._started_ms = int(time.time() * 1000)
            self._message_count = 0
            self._topic_filter = {t for t in (topics or []) if t} or None
            self._registered_topics = set()

        self._gateway.set_raw_sink(self._on_raw_message)
        return self.status()

    def _on_raw_message(
        self,
        topic: str,
        msg_type: str,
        raw_bytes: bytes,
        timestamp_ns: int,
    ) -> None:
        import rosbag2_py

        with self._lock:
            if not self._active or self._writer is None:
                return
            if self._topic_filter is not None and topic not in self._topic_filter:
                return
            if topic not in self._registered_topics:
                self._writer.create_topic(rosbag2_py.TopicMetadata(
                    name=topic,
                    type=msg_type,
                    serialization_format='cdr',
                ))
                self._registered_topics.add(topic)
            self._writer.write(topic, raw_bytes, int(timestamp_ns))
            self._message_count += 1

    def stop(self) -> RecordingInfo:
        self._gateway.set_raw_sink(None)
        with self._lock:
            if not self._active or self._name is None or self._bag_dir is None:
                raise FileNotFoundError('当前没有正在进行的录制')
            name = self._name
            bag_dir = self._bag_dir
            duration_s = round(max(0.0, time.monotonic() - self._started_mono), 3)
            msg_count = self._message_count
            topics = sorted(self._registered_topics)
            created_at_ms = self._started_ms
            writer = self._writer
            self._active = False
            self._name = None
            self._bag_dir = None
            self._writer = None
            self._topic_filter = None
            self._registered_topics = set()

        if writer is not None:
            del writer

        size_bytes = sum(p.stat().st_size for p in bag_dir.rglob('*') if p.is_file())
        info = RecordingInfo(
            name=name,
            storage_format=self._storage_format,
            size_bytes=size_bytes,
            duration_s=duration_s,
            message_count=msg_count,
            topics=topics,
            created_at_ms=created_at_ms,
        )
        meta_path = self._dir / f'.{name}.meta.json'
        meta_path.write_text(info.model_dump_json(), encoding='utf-8')
        return info

    def list_recordings(self) -> list[RecordingInfo]:
        out: list[RecordingInfo] = []
        active_name = self._name if self._active else None
        for entry in sorted(self._dir.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True):
            if not entry.is_dir() or entry.name.startswith('.') or entry.name == active_name:
                continue
            meta_path = self._dir / f'.{entry.name}.meta.json'
            if meta_path.is_file():
                try:
                    out.append(RecordingInfo.model_validate_json(meta_path.read_text(encoding='utf-8')))
                    continue
                except Exception:
                    pass
            mcap_files = list(entry.glob('*.mcap'))
            fmt: Literal['mcap', 'sqlite3'] = 'mcap' if mcap_files else 'sqlite3'
            size_bytes = sum(p.stat().st_size for p in entry.rglob('*') if p.is_file())
            out.append(RecordingInfo(
                name=entry.name,
                storage_format=fmt,
                size_bytes=size_bytes,
                duration_s=0.0,
                message_count=0,
                topics=[],
                created_at_ms=int(entry.stat().st_mtime * 1000),
            ))
        return out

    def resolve_download(self, name: str) -> tuple[Path, str, str]:
        bag_dir = self._safe_bag_dir(name)
        if not bag_dir.is_dir():
            raise FileNotFoundError(f'录制 {name} 不存在')
        if self._active and self._name == name:
            raise RuntimeError('录制尚未停止，不能下载')
        mcap_files = sorted(bag_dir.glob('*.mcap'))
        if mcap_files:
            return mcap_files[0], f'{name}.mcap', 'application/octet-stream'
        zip_path = self._dir / f'.{name}.zip'
        with zipfile.ZipFile(zip_path, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
            for item in sorted(bag_dir.rglob('*')):
                if item.is_file():
                    zf.write(item, arcname=f'{name}/{item.relative_to(bag_dir).as_posix()}')
        return zip_path, f'{name}.zip', 'application/zip'

    def delete(self, name: str) -> None:
        bag_dir = self._safe_bag_dir(name)
        if self._active and self._name == name:
            raise RuntimeError('不能删除正在录制的会话')
        if not bag_dir.exists():
            raise FileNotFoundError(f'录制 {name} 不存在')
        shutil.rmtree(bag_dir)
        for aux in (self._dir / f'.{name}.meta.json', self._dir / f'.{name}.zip'):
            if aux.exists():
                aux.unlink()
