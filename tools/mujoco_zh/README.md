# MuJoCo 3.6 中文 Viewer

本工具将原生菜单、按钮、参数标签和常用帮助文字显示为中文。关节名、模型标识符、数值及快捷键保留原值。部分动态状态、图表标题和底层错误信息仍显示英文。

## 启动

直接打开当前工程的整机模型。

```bash
/home/ly/robot2_ws/tools/mujoco_zh/run.sh
```

打开其他模型。

```bash
/home/ly/robot2_ws/tools/mujoco_zh/run.sh --mjcf /absolute/path/model.xml
```

已有 Python Viewer 程序可使用汉化运行环境启动。

```bash
/home/ly/robot2_ws/tools/mujoco_zh/python_zh.sh /absolute/path/viewer_script.py
```

这是独立入口，运行包放在 `~/.cache/robot2-mujoco-zh/runtime`。工作台的自动启动入口仍使用原来的环境，已打开的窗口需要通过上述入口重新打开才能显示中文。

## 构建和检查

使用本机已安装 MuJoCo 3.6.0 的 Python。构建需要 Git、GCC/G++、pkg-config、FreeType 开发文件和 Noto CJK 字体，首次构建需要下载同版本官方源码。

```bash
/home/ly/.ros/ros2_control/.venv/bin/python /home/ly/robot2_ws/tools/mujoco_zh/build.py
/home/ly/robot2_ws/tools/mujoco_zh/python_zh.sh /home/ly/robot2_ws/tools/mujoco_zh/verify.py
```

检查覆盖原生控件标签、六档字号、快捷键和滑块事件，截图写入缓存目录的 `native-ui-*.png`。验证额外使用已安装的 NumPy、Pillow 和 EGL。

## 实现范围

构建脚本复制现有 Python 包，并重新编译同版本的经典渲染器和 UI。原生 UI 在绘制时查找 `translations.json`，FreeType 提供中文字形。复制的原版物理库作为依赖加载，绘制层通过同长度的 ELF SONAME 替换区分两份库。

运行环境固定为 MuJoCo 3.6.0 和本机 Python 路径。升级 MuJoCo 时需要重新核对上游源码补丁、ABI 和数值一致性。中文阴影文字目前采用普通字形绘制。每个渲染线程的字形缓存保留到进程退出。

上游代码遵循 Apache-2.0，构建包附带 `MUJOCO-LICENSE`。
