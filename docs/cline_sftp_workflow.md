# 本地 Cline + SFTP + 树莓派工作流

## 目标

让 Cline 运行在 Windows 本地，避免 VS Code Remote SSH 窗口中的扩展宿主占用树莓派 CPU/内存，同时仍然可以快速把 ROS1 上位机代码同步到树莓派运行。

## 推荐双窗口

### 窗口 1：Windows 本地 SFTP 工作区

```text
打开目录：D:\cpprobot\pi
用途：Cline 写代码、改文档、生成 ROS 包
启用插件：Cline、SFTP
```

### 窗口 2：Remote SSH 树莓派窗口

```text
连接目标：ubuntu@100.124.15.24，网线备用 ubuntu@192.168.137.100
用途：运行 roscore、catkin_make、rosrun、rostopic、串口测试
启用插件：Remote SSH
不要启用：Cline
```

## 同步关系

推荐只同步源码目录：

```text
Windows: D:\cpprobot\pi\src
树莓派: /home/ubuntu/tao_robot_ws/src
```

树莓派上的 `build/`、`devel/`、`log/` 由 `catkin_make` 自动生成，不同步回 Windows，也不提交 Git。

## 连接优先级

PowerShell 脚本默认优先使用 Tailscale：

```text
优先：100.124.15.24
备用：192.168.137.100
```

也就是说，执行下面命令时会先测试 Tailscale SSH；如果不通，再尝试网线 IP：

```powershell
.\scripts\sync_to_pi.ps1
.\scripts\pi_exec.ps1 "hostname && pwd"
```

SFTP 插件通常不支持自动 fallback，所以 `sftp_config.example.json` 默认填写 Tailscale IP。如果 Tailscale 临时不可用，可以手动把 `host` 改为网线 IP：

```json
"host": "192.168.137.100"
```

当前 ROS 节点都在树莓派本机运行，SSH/SFTP 使用 Tailscale 或网线不会影响 ROS 本机通信，因此不需要修改 `ROS_MASTER_URI` 或 `ROS_IP`。

## 为什么不要在 Remote SSH 窗口用 Cline

Remote SSH 工作区中的扩展经常运行在远程扩展宿主上。如果在树莓派远程窗口启用 Cline，容易导致树莓派 CPU/内存占用过高，出现卡顿、无响应或上下文处理失败。

正确做法：

```text
Cline 只在 Windows 本地窗口使用
Remote SSH 只负责终端运行和查看真实环境
```

## 验证同步

1. 在 Windows 本地 `D:\cpprobot\pi\src` 新建测试文件。
2. 保存后等待 SFTP 上传。
3. 在 Remote SSH 终端执行：

```bash
ls /home/ubuntu/tao_robot_ws/src
```

如果能看到文件，说明同步链路正常。
