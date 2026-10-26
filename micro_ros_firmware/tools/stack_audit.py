#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务栈静态审计：把「大局部变量 vs 任务栈」从人工清点变成可复跑的证据。

由 2026-09-18 台架的 storage 栈溢出引出（`perform_save()` 在 4KB 栈上放 5,120 B
的局部数组 → FreeRTOS 溢出钩子关中断死循环 → 表现为 IWDG 复位、**没有任何显式
线索**）。本脚本用 GCC 的 `-fstack-usage` + `-fcallgraph-info` 产物，按任务入口
算出**最坏调用路径的栈深度**，与 `xTaskCreate` 声明的栈比。

**需要一次带插桩的构建**（产物 `.su` / `.ci` 只在开启这些 flag 后才生成）：

    # platformio.ini 的 build_flags 里临时加两行（勿提交）：
    #     -fstack-usage
    #     -fcallgraph-info=su,da
    ~/.platformio/penv/bin/pio run -t clean && ~/.platformio/penv/bin/pio run
    python3 tools/stack_audit.py --build .pio/build/stm32h723vgt6

用法：
    python3 tools/stack_audit.py                     # 默认表 + 默认构建目录
    python3 tools/stack_audit.py --list              # 只列各任务路径前 10 名
    python3 tools/stack_audit.py --task motor_rt_task_fn

已知盲区（结论要按这个打折，别当成保证）：
  - **间接调用（函数指针）不在图里**：adapter 的 `config_*` 虚表、FreeRTOS 的
    入口/钩子、HAL 回调都是运行时才确定，走不到的就漏。本板的最大间接面是
    `motor_protocol_adapter_t` 与 FreeRTOS 调度本身。
  - `.su` 的数字不含调用约定额外开销（寄存器入栈、8 字节对齐填充），
    经验上单帧再加 8~16 B。
  - `dynamic`（alloca/VLA）与 `bounded`（GCC 算出的上界）需按语义分别累加。
  - 只在**单核、无中断嵌套**口径下成立；ISR 用的是 MSP，不占任务栈。
"""

from __future__ import annotations

import argparse
import re
from collections import defaultdict
from pathlib import Path

# 任务入口 → 声明栈（字节）。xTaskCreate 的深度参数单位是 word（4 B）。
# 与 src/*.cpp 的栈常量对应，修改时须同步。
TASK_STACKS = {
    "start_task": 256 * 4,
    "led_task_fn": 256 * 4,
    "sensor_task_fn": 512 * 4,
    "motor_rt_task_fn": 1024 * 4,
    "storage_task_fn": 1024 * 4,
    "micro_ros_task_fn": 8192 * 4,
}

NODE_RE = re.compile(r'^node:\s*\{\s*title:\s*"([^"]*)"\s*label:\s*"(.*?)"\s*(.*)\}\s*$')
EDGE_RE = re.compile(r'^edge:\s*\{\s*sourcename:\s*"([^"]*)"\s*targetname:\s*"([^"]*)"')


def parse_su(build: Path) -> dict[str, tuple[int, str, str]]:
    """函数签名的可读形式 → (栈字节, 限定符, 位置)；重复取最大。"""
    out: dict[str, tuple[int, str, str]] = {}
    for f in build.rglob("*.su"):
        for line in f.read_text(errors="replace").splitlines():
            parts = line.split("\t")
            if len(parts) < 3:
                continue
            loc, size, qual = parts[0], parts[1], parts[2]
            # loc = 路径:行:列:函数签名
            sig = loc.split(":", 3)[3] if loc.count(":") >= 3 else loc
            try:
                n = int(size)
            except ValueError:
                continue
            if sig not in out or n > out[sig][0]:
                out[sig] = (n, qual, ":".join(loc.split(":")[:2]))
    return out


def parse_ci(build: Path):
    """(title→label 首行) 与邻接表；title 是符号名，label 首行是可读签名。"""
    defs: dict[str, str] = {}
    edges: dict[str, set[str]] = defaultdict(set)
    for f in build.rglob("*.ci"):
        for line in f.read_text(errors="replace").splitlines():
            m = NODE_RE.match(line)
            if m:
                title, label = m.group(1), m.group(2)
                defs[title] = label.split("\\n")[0]
                continue
            m = EDGE_RE.match(line)
            if m:
                edges[m.group(1)].add(m.group(2))
    return defs, edges


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build", default=".pio/build/stm32h723vgt6")
    ap.add_argument("--task", action="append", default=None)
    ap.add_argument("--list", action="store_true", help="打印每个任务的路径前列")
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--root", action="append", default=None,
                    help="额外起点（间接调用/回调入口），可与 --task 并存")
    ap.add_argument("--orphans", type=int, default=0,
                    help="打印前 N 个「无静态调用者」的大帧（只能经函数指针到达）")
    args = ap.parse_args()

    build = Path(args.build)
    su = parse_su(build)
    defs, raw_edges = parse_ci(build)
    if not su or not defs:
        raise SystemExit(f"{build} 下没有 .su/.ci——先按 docstring 加 flag 重新构建")

    # 把 title（符号）归一到可读签名，与 .su 的键对齐。
    label_of = {t: defs.get(t, t) for t in set(defs) | {e for s in raw_edges.values() for e in s}}
    edges: dict[str, set[str]] = defaultdict(set)
    for src, dsts in raw_edges.items():
        for d in dsts:
            edges[label_of.get(src, src)].add(label_of.get(d, d))
    for sig in su:
        edges.setdefault(sig, set())

    # 入口签名归一：TASK_STACKS 里给的是可读名，实际签名带参数表。
    def find_entry(name: str) -> str | None:
        # .su 的键是**带返回类型**的可读签名（`void motor_rt_task_fn(void*)`），
        # 不是裸符号名——按 `名字(` 匹配，不能按前缀匹配。
        for sig in su:
            if f"{name}(" in sig or sig == name:
                return sig
        return None

    callers: dict[str, list[str]] = defaultdict(list)
    for src, dsts in edges.items():
        for d in dsts:
            callers[d].append(src)

    def worst_from(entry: str) -> tuple[int, tuple[str, ...]]:
        """带记忆的 DFS：单条路径上累加，取最坏路径（环按 0 处理）。"""
        memo: dict[str, tuple[int, tuple[str, ...]]] = {}
        on_path: set[str] = set()

        def walk(sig: str) -> tuple[int, tuple[str, ...]]:
            if sig in memo:
                return memo[sig]
            if sig in on_path:
                return 0, (sig + " [环]",)
            on_path.add(sig)
            size = su.get(sig, (0, "", ""))[0]
            best, best_path = 0, ()
            for callee in edges.get(sig, ()):
                cs, cp = walk(callee)
                if cs > best:
                    best, best_path = cs, cp
            on_path.discard(sig)
            res = (size + best, (sig,) + best_path)
            memo[sig] = res
            return res

        return walk(entry)

    print(f"构建目录 {build}：{len(su)} 个函数有栈用量，{len(raw_edges)} 个调用点\n")

    if args.orphans:
        print(f"无静态调用者的大帧（只能经函数指针/回调到达，逐任务数字**不覆盖**）：")
        cand = [(su[k][0], k, su[k][2]) for k in su if not callers.get(k)]
        for size, sig, loc in sorted(cand, reverse=True)[:args.orphans]:
            print(f"  {size:>5} B  {sig[:78]}  ({loc})")
        print()
    worst = []
    for name, stack_bytes in TASK_STACKS.items():
        entry = find_entry(name)
        if entry is None:
            print(f"!! 未找到任务入口 {name}（构建产物不含它？）")
            continue
        total, path = worst_from(entry)
        worst.append((total, stack_bytes, name, path))
        pct = 100.0 * total / stack_bytes
        flag = " **超出**" if total > stack_bytes else (" 偏紧" if pct > 70 else "")
        print(f"== {name}：最坏 {total} B / 声明 {stack_bytes} B（{pct:.1f}%）{flag}")
        if args.list or args.task:
            for i, sig in enumerate(path[:args.top]):
                mark = "└" if i == len(path[:args.top]) - 1 else "├"
                print(f"   {mark} {su.get(sig, (0, '', ''))[0]:>5} B  {sig[:96]}")
        print()

    for name in (args.root or []):
        entry = find_entry(name)
        if entry is None:
            print(f"!! 未找到起点 {name}")
            continue
        total, path = worst_from(entry)
        print(f"== [间接起点] {entry[:70]}：最坏 {total} B")
        for i, sig in enumerate(path[:args.top]):
            print(f"   ├ {su.get(sig, (0, '', ''))[0]:>5} B  {sig[:92]}")
        print()

    print("汇总（最坏路径 / 声明栈）：")
    for total, stack, name, _ in sorted(worst, key=lambda x: x[0] / x[1], reverse=True):
        print(f"  {100.0 * total / stack:6.1f}%   {total:>6} / {stack:>6} B   {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
