"""`systemd_units` 的单测。

不启任何真 unit：全部走注入的假 runner。本文件要钉住的是**四态判定与启动三段分支**——
它们是「状态权威 = systemd」能否落地的地方，也是实测踩过的地方（见模块 docstring）。
"""

from __future__ import annotations

from conftest import FakeSystemd

from bench_manager.systemd_units import (
    AUTO_STARTED_ENV,
    SLICE,
    UNIT_AGENT,
    UNIT_MOTOR_BACKEND,
    UNIT_SIMULATION,
    SystemdUnits,
    UnitState,
    ros_environment,
)


def units(**properties: str) -> SystemdUnits:
    return SystemdUnits(runner=FakeSystemd({UNIT_AGENT: properties}))


# --- 四态判定：LoadState 必须一起看 -----------------------------------------

def test_absent_unit_is_not_mistaken_for_a_cleanly_stopped_one() -> None:
    """不存在和已停止的 unit 均可为 inactive/dead/0，须用 LoadState 区分。"""
    assert units(LoadState='not-found', ActiveState='inactive',
                 SubState='dead', ExecMainStatus='0').probe(UNIT_AGENT) \
        == (UnitState.ABSENT, None)


def test_stopped_unit_is_stopped() -> None:
    assert units(LoadState='loaded', ActiveState='inactive',
                 SubState='dead', ExecMainStatus='0').probe(UNIT_AGENT) \
        == (UnitState.STOPPED, None)


def test_running_unit_is_running() -> None:
    assert units(LoadState='loaded', ActiveState='active',
                 SubState='running', ExecMainStatus='0').probe(UNIT_AGENT) \
        == (UnitState.RUNNING, None)


def test_failed_unit_carries_the_exit_code() -> None:
    """failed 态要把退出码带出来——页面靠它显示「exit N」。"""
    assert units(LoadState='loaded', ActiveState='failed',
                 SubState='failed', ExecMainStatus='17').probe(UNIT_AGENT) \
        == (UnitState.FAILED, 17)


# --- 启动：三段分支 ---------------------------------------------------------

def test_start_on_absent_unit_creates_it_with_systemd_run() -> None:
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found',
                                        'ActiveState': 'inactive'}})
    SystemdUnits(runner=systemd).start(
        UNIT_AGENT, ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent'], auto=False)

    create = [c for c in systemd.commands()
              if c[:3] == ['systemd-run', '--user', '--unit=' + UNIT_AGENT]]
    assert len(create) == 1, systemd.commands()
    assert f'--slice={SLICE}' in create[0]
    assert create[0][-4:] == ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent']


def test_start_never_collects_the_unit() -> None:
    """**不加 `--collect`**：加了会在进程退出后卸载 unit，ExecMainStatus 随之消失，
    页面就报不出「退出码 N」。残留由启动前的 reset-failed 清。
    """
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found',
                                        'ActiveState': 'inactive'}})
    SystemdUnits(runner=systemd).start(UNIT_AGENT, ['true'], auto=False)

    assert not any('--collect' in c for c in systemd.commands())


def test_start_on_running_unit_adopts_without_touching_it() -> None:
    """认领：已在跑就什么都不做。**不重启**——重启会打断正在跑的仿真。

    这也是 systemd 免费给的"单写者所有权"：unit 名被占用时第二个 bench_manager
    根本起不来，所以"在跑"就意味着"有人管着"。
    """
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'loaded',
                                        'ActiveState': 'active',
                                        'SubState': 'running'}})
    SystemdUnits(runner=systemd).start(UNIT_AGENT, ['true'], auto=False)

    assert [c[1:3] for c in systemd.commands()] == [['--user', 'show']], systemd.commands()


def test_start_on_failed_unit_resets_before_restarting() -> None:
    """failed 的 unit 名字仍被占着，不先 reset-failed 就直接 systemd-run 会报
    "Unit already exists"——这是「重启」路径最容易踩的一步。
    """
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'loaded',
                                        'ActiveState': 'failed'}})
    SystemdUnits(runner=systemd).start(UNIT_AGENT, ['true'], auto=False)

    verbs = [c[1:4] for c in systemd.commands() if c[0] == 'systemctl']
    assert ['--user', 'reset-failed', UNIT_AGENT] in verbs, systemd.commands()


def test_start_on_stopped_unit_recreates_it_with_the_current_argv() -> None:
    """停着的 unit 也**重建**，不走 `systemctl start`。

    `systemctl start` 跑的是 unit 里存着的**旧** ExecStart——换个模型或接口再启动时，
    新 argv 会被静默忽略，表现成"改了配置没生效"。所以除了 RUNNING（认领），一律按当前
    argv 重建。
    """
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'loaded',
                                        'ActiveState': 'inactive'}})
    SystemdUnits(runner=systemd).start(UNIT_AGENT, ['true'], auto=False)

    commands = systemd.commands()
    assert not any(c[1:4] == ['--user', 'start', UNIT_AGENT] for c in commands)
    assert any(c[:2] == ['systemd-run', '--user'] for c in commands), commands


# --- 停止：exit 5 是"已停"，不是错误 ----------------------------------------

def test_stop_treats_not_loaded_as_success() -> None:
    """停止不存在的 service 返回 exit 5，须视为成功，避免生命周期卡在 deactivating。"""
    systemd = FakeSystemd(fail={('systemctl', '--user', 'stop', UNIT_AGENT):
                                (5, 'Failed to stop bench-agent.service: '
                                    'Unit bench-agent.service not loaded.')})
    SystemdUnits(runner=systemd).stop(UNIT_AGENT)  # 不该抛


def test_stop_slice_targets_the_bench_slice() -> None:
    systemd = FakeSystemd()
    SystemdUnits(runner=systemd).stop_slice()

    assert ['systemctl', '--user', 'stop', SLICE] in systemd.commands()


def test_stop_slice_clears_failed_units_too() -> None:
    """失败退出的 unit 不是「活跃」的，`stop` 停不掉它——必须另外清。

    留着的话，下一次 `on_activate` 会把它认领成「上一轮的会话」，页面顶一条早就过期
    的失败（实测踩到：Ctrl-C 之后 bench-simulation.service 仍是 loaded/failed）。
    """
    systemd = FakeSystemd({UNIT_SIMULATION: {
        'LoadState': 'loaded', 'ActiveState': 'failed', 'SubState': 'failed',
        'ExecMainStatus': '1'}})

    SystemdUnits(runner=systemd).stop_slice()

    assert systemd.properties[UNIT_SIMULATION]['LoadState'] == 'not-found'
    for unit in (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT):
        assert ['systemctl', '--user', 'reset-failed', unit] in systemd.commands()


def test_stop_slice_is_safe_when_the_slice_never_existed() -> None:
    """首次运行（一个 unit 都没起过）时 slice 不存在，`stop bench.slice` 返回 exit 0。"""
    systemd = FakeSystemd(fail={('systemctl', '--user', 'stop', SLICE): (0, '')})
    SystemdUnits(runner=systemd).stop_slice()  # 不该抛


# --- 环境白名单 -------------------------------------------------------------

def test_ros_environment_keeps_only_whitelisted_entries() -> None:
    """transient service 跑在 "clean and detached execution environment" 里，不继承
    调用者环境（实测：user manager 的 47 个变量里零个 ROS 变量）。ROS 的发现路径
    只能靠白名单显式带进去，否则 unit 里的 `ros2 launch` 找不到包。
    """
    kept = ros_environment({
        'AMENT_PREFIX_PATH': '/opt/ros/humble:/ws/install/motor_sim',
        'ROS_DOMAIN_ID': '0',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'LD_LIBRARY_PATH': '/opt/ros/humble/lib',
        'PATH': '/usr/bin',
        'PYTHONPATH': '/opt/ros/humble/lib/python3.10/site-packages',
        'EDITOR': 'vim',
        'SSH_AUTH_SOCK': '/tmp/agent',
    })

    assert kept['AMENT_PREFIX_PATH'].endswith('install/motor_sim')
    assert kept['ROS_DOMAIN_ID'] == '0'
    assert kept['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert kept['LD_LIBRARY_PATH'] == '/opt/ros/humble/lib'
    assert kept['PATH'] == '/usr/bin'
    assert 'EDITOR' not in kept
    assert 'SSH_AUTH_SOCK' not in kept


def test_start_passes_the_ros_environment_into_the_unit() -> None:
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found'}})
    SystemdUnits(runner=systemd,
                 environ={'AMENT_PREFIX_PATH': '/ws/install/x',
                          'EDITOR': 'vim'}).start(UNIT_AGENT, ['true'], auto=True)

    create = next(c for c in systemd.commands() if c[0] == 'systemd-run')
    assert '--setenv=AMENT_PREFIX_PATH=/ws/install/x' in create
    assert not any('EDITOR' in arg for arg in create)


def test_auto_ownership_is_persisted_into_the_unit() -> None:
    """auto 归属要落进 unit 的环境，否则 bench_manager 一重启就把「随仿真起的」
    误判成「手动起的」，停仿真时不敢停它。
    """
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found'}})
    SystemdUnits(runner=systemd).start(UNIT_AGENT, ['true'], auto=True)

    create = next(c for c in systemd.commands() if c[0] == 'systemd-run')
    assert f'--setenv={AUTO_STARTED_ENV}=1' in create


def test_session_description_is_persisted_into_the_unit() -> None:
    """会话自身的描述（哪台设备/哪个模型/哪条总线）随 unit 持久化。

    以前这些活在 manager 的内存里，bench_manager 一重启就丢——页面只能显示「在跑」，
    说不清在跑什么。
    """
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found'}})
    SystemdUnits(runner=systemd).start(
        UNIT_AGENT, ['true'], auto=False,
        session_env={'BENCH_DEVICE': '/dev/serial/by-id/usb-STMicro-if00'})

    create = next(c for c in systemd.commands() if c[0] == 'systemd-run')
    assert '--setenv=BENCH_DEVICE=/dev/serial/by-id/usb-STMicro-if00' in create


def test_unit_environment_is_readable_for_ownership_checks() -> None:
    systemd = FakeSystemd({UNIT_MOTOR_BACKEND: {
        'LoadState': 'loaded', 'ActiveState': 'active',
        'Environment': f'{AUTO_STARTED_ENV}=1 BENCH_CONFIG=abc123'}})
    unit = SystemdUnits(runner=systemd)

    assert unit.environment(UNIT_MOTOR_BACKEND)[AUTO_STARTED_ENV] == '1'
    assert unit.environment(UNIT_MOTOR_BACKEND)['BENCH_CONFIG'] == 'abc123'


# --- 可用性探测 -------------------------------------------------------------

def test_availability_is_false_without_a_user_manager() -> None:
    """没有 user manager / session bus 时要能提前发现（configure 阶段 fail-fast），
    而不是等到运行期每次调用都失败、节点行为未定义。
    """
    systemd = FakeSystemd(fail={('systemctl', '--user', 'show-environment'):
                                (1, 'Failed to connect to bus: No such file')})
    assert SystemdUnits(runner=systemd).available() is False


def test_availability_is_true_with_a_user_manager() -> None:
    assert SystemdUnits(runner=FakeSystemd()).available() is True


def test_unit_names_are_the_contract() -> None:
    assert (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT) == (
        'bench-simulation.service', 'bench-motor-backend.service',
        'bench-agent.service')
    assert SLICE == 'bench.slice'
