# GPT-Live 本地语音工具

Python 3 标准库 + 浏览器 WebRTC，无需安装依赖。

## Windows 单文件版

将 `dist/GPT-Live-Windows-x64.exe` 发给对方即可，支持 Windows 10/11 x64，
无需安装 Python。双击后自动打开默认浏览器，在页面填写自己的 OpenAI API 密钥。
保持启动的控制台窗口打开；结束对话后在控制台按 Ctrl+C 退出。
每次使用随机空闲端口，避免多次启动时端口冲突。

程序内嵌官方 Python 3.14.10 运行时（包含其许可证）和三个应用文件，
不包含开发者密钥。运行时释放到临时目录，正常退出后删除。
程序未做 Windows 代码签名，也未在 Windows 真机验收；系统可能显示未知发布者提示。

重新打包需安装 Go，Linux 和 Windows 均可构建：

```bash
python3 tools/gpt_live/build_windows.py
```

构建脚本下载并校验官方 Windows Python ZIP，通过 Go 交叉编译启动器，
先运行解包测试，再验证输出的 Windows PE x64 文件头，并生成 SHA-256 校验文件。

## Python 源码版

```bash
cd /home/ly/robot2_ws
python3 tools/gpt_live/server.py
```

打开 http://localhost:8765 ，点击「开始对话」，允许麦克风后用中文交流。
页面可填写自己的 OpenAI API 密钥，留空使用服务端默认密钥。
填写的密钥只用于当前连接请求，固定发送到 OpenAI 官方接口，不写入文件或
浏览器存储；刷新页面需重新填写。没有本机默认密钥也可启动服务。
支持麦克风静音、助手音频播放、结束会话及连接事件查看。建议使用耳机。
不提供本地文件访问、命令执行或机器人控制功能，也不保存录音。

页面留空时，服务端优先使用环境变量 `OPENAI_API_KEY`；未设置时，只读复用
`~/.cc-switch/cc-switch.db` 中 Codex 的 `OpenAI API` 配置，且仅接受官方地址。
服务端默认密钥不会发送到浏览器、写入源码或日志。也可在终端安全输入后启动：

```bash
read -rsp 'OpenAI API key: ' OPENAI_API_KEY
export OPENAI_API_KEY
python3 tools/gpt_live/server.py
```

环境变量 `OPENAI_BASE_URL` 可指定支持 `/live/sessions` 的网关（仅用于环境密钥）；
`OPENAI_BACKEND_MODEL` 可覆盖委派后台模型，默认采用官方示例的 `gpt-5.6-terra`。
语音模型固定为 `gpt-live-1`，不使用 CC Switch 的 `ccs-openai/` 路由前缀。
端口冲突时使用 `--port 8766`。

语音按会话时长收费，初始化也可能产生费用，后台模型另计。静音不会结束计费。
正常结束会发送 `session.close` 并等待 `session.closed`；网络断开或关闭页面时
只能尽力清理连接，无法保证收到最终用量。不要将本地演示服务开放到公网。
HTTP 401/403 通常需要检查密钥和模型权限；404 检查网关 Live 接口支持；
429 检查额度或限流。接口错误不会自动重试，以免重复创建收费会话。

验证：

```bash
python3 -m unittest discover -s tools/gpt_live -p 'test_*.py' -v
node --test tools/gpt_live/test_app.mjs
```

接口依据：[官方 GPT-Live WebRTC 指南](https://developers.openai.com/api/docs/guides/voice-webrtc?api=live)。
