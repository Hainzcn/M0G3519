# M0G3519 工程 — EIDE 配置与编译说明

本文档说明 **BallBalanceRover** 项目中 MSPM0G3519 固件如何在 **VS Code / Cursor + EIDE** 下配置与编译。

目标芯片：**MSPM0G3519**（Cortex-M0+）  
EIDE 工程目录：`M0G3519/keil/`  
应用源码：`M0G3519/src/`（`main.c`、各层模块）  
SysConfig 配置：`M0G3519/M0G3519.syscfg`  
SysConfig 生成文件：`M0G3519/keil/`（`ti_msp_dl_config.c/h`）

---

## 1. 环境准备

请在本机安装以下工具，并确认路径可用：

| 工具 | 用途 | 示例路径 |
|------|------|----------|
| [EIDE 扩展](https://marketplace.visualstudio.com/items?itemName=cl.eide) | 嵌入式构建 | VS Code / Cursor 扩展 |
| Keil MDK (ARM Compiler 6) | 编译 / 链接 | `A:/Program Files (x86)/Keil_v5/ARM/ARMCLANG` |
| MSPM0 SDK | 头文件、DriverLib、SysConfig 元数据 | `A:/ti/mspm0_sdk_2_10_00_04` |
| SysConfig | 根据 `.syscfg` 生成 `ti_msp_dl_config.*` | `A:/ti/sysconfig_1.28.0/sysconfig_cli.bat` |
| XDS110（可选） | 烧录 | `A:/ti/XDS110_3519/dslite-CORTEX_M0P.bat` |

打开工程：

```text
M0G3519/keil/M0G3519_nortos_keil.code-workspace
```

在 EIDE 中配置 Keil 工具链路径：**Settings → EIDE → ARM.AC6 → Keil 安装目录**。

---

## 2. EIDE 工程配置

主配置文件：`.eide/eide.yml`  
构建目标名：**M0G3519_nortos_keil**  
工具链：**AC6**（Keil ARM Compiler 6）

### 2.1 源码结构（virtualFolder）

| 分组 | 路径 | 说明 |
|------|------|------|
| Source | `../src/main.c`、`../M0G3519.syscfg`、启动与 SysConfig 生成文件 | 入口与硬件描述 |
| Hardware | `../src/hardware/*` | 外设驱动层 |
| Middle | `../src/middle/*` | 控制与业务逻辑 |
| App | `../src/app/*` | 应用任务 |
| Libraries | `../src/MSPM0G3519_Library/*` | 逐飞库封装 |

### 2.2 预构建任务（SysConfig）

构建前自动运行 `syscfg.bat`，由 SysConfig 生成 `ti_msp_dl_config.c/h`：

```yaml
beforeBuildTasks:
  - name: linking syscfg
    abortAfterFailed: true
    command: '"${projectRoot}/syscfg.bat" "${projectRoot}" M0G3519.syscfg'
    disable: false
    stopBuildAfterFailed: true
```

说明：

- 使用 EIDE 变量 `${projectRoot}`，**不要**使用 Keil uVision 的 `$P`（EIDE 不会展开）。
- `.syscfg` 位于 `M0G3519/M0G3519.syscfg`；生成文件输出到 `keil/`（`-o "%PROJ_DIR%"`）。

### 2.3 头文件搜索路径（incList）

```yaml
incList:
  - .
  - ..
  - ../src/hardware
  - ../src/middle
  - ../src/app
  - ../src/MSPM0G3519_Library/zf_common
  - ../src/MSPM0G3519_Library/zf_driver
  - <SDK_ROOT>/source
  - <SDK_ROOT>/source/third_party/CMSIS/Core/Include
  - .cmsis/include
  - RTE/_M0G3519_nortos_keil
```

- `.` 对应 `keil/`，供 `ti_msp_dl_config.h` 等生成文件使用。
- `..` 对应 `M0G3519/`，供 `M0G3519.syscfg` 等上级目录文件引用。
- Windows 路径在 `eide.yml` 中建议使用 **正斜杠** `/`。

### 2.4 编译与链接选项

| 项 | 值 |
|----|-----|
| CPU | Cortex-M0+ |
| C 标准 | C99 |
| 优化 | `-O2`（level-2） |
| 宏定义 | `__MSPM0G3519__` |
| 短 enum / wchar | `short-enums#wchar: true`（等价 `-fshort-enums -fshort-wchar`） |
| Scatter | `./mspm0g3519.sct` |
| DriverLib | `<SDK_ROOT>/source/ti/driverlib/lib/keil/m0p/mspm0gx51x/driverlib.a` |

链接 SDK 预编译 `driverlib.a` 时必须开启短 enum / 短 wchar，否则会报 `L6242E`。

### 2.5 烧录配置（可选）

当前 uploader 为 **Custom**，使用 XDS110：

```yaml
commandLine: '"A:/ti/XDS110_3519/dslite-CORTEX_M0P.bat" --config="A:/ti/XDS110_3519/user_files/configs/MSPM0G3519.ccxml" -e -f -v "${ExecutableName}.hex"'
```

路径需按本机调试器安装位置修改。

---

## 3. 本机路径配置

以下两处需替换为你本机实际安装位置：

- `<SDK_ROOT>` — MSPM0 SDK 根目录，例如 `A:/ti/mspm0_sdk_2_10_00_04`
- `<SYSCONFIG_CLI>` — SysConfig 命令行入口，例如 `A:/ti/sysconfig_1.28.0/sysconfig_cli.bat`

### 3.1 修改 `syscfg.bat`

```bat
set "SYSCFG_CLI=A:\ti\sysconfig_1.28.0\sysconfig_cli.bat"
set "SDK_ROOT=A:\ti\mspm0_sdk_2_10_00_04"
```

确认 `<SDK_ROOT>/.metadata/product.json` 存在。

### 3.2 修改 `.eide/eide.yml`

将 `incList`、`linker.misc-controls` 及烧录命令中的 SDK / 调试器路径改为本机路径（见 2.3、2.4、2.5）。

修改编译选项或路径后，建议 **Clean → Rebuild** 全量重编。

---

## 4. 编译指令

### 4.1 在 IDE 中编译

1. 打开 `M0G3519_nortos_keil.code-workspace`。
2. 在 EIDE 侧边栏选择目标 **M0G3519_nortos_keil**。
3. 使用快捷键或 EIDE 工具栏：

| 操作 | 快捷键 |
|------|--------|
| 构建（增量） | **F7** |
| 重新构建（全量） | **Ctrl+Alt+F7** |
| 清理 | **Ctrl+Alt+D** |

正常构建日志顺序：

1. `pre-build tasks` → SysConfig 成功
2. `compiling` → 编译 `.c` / 汇编
3. `linking` → 生成 `.axf` / `.hex`

### 4.2 命令行编译（unify_builder）

EIDE 底层通过 **unify_builder** 读取 `builder.params` 驱动 Keil AC6 编译。可在工程目录下手动执行：

```powershell
cd M0G3519\keil

# 增量构建
& "$env:USERPROFILE\.cursor\extensions\cl.eide-*\res\tools\win32\unify_builder\unify_builder.exe" `
  -p "build\M0G3519_nortos_keil\builder.params"

# 全量重建
& "$env:USERPROFILE\.cursor\extensions\cl.eide-*\res\tools\win32\unify_builder\unify_builder.exe" `
  -p "build\M0G3519_nortos_keil\builder.params" --rebuild
```

说明：

- `builder.params` 在 EIDE 中点击构建时自动生成/更新；修改 `eide.yml` 后建议先在 IDE 中 Build 一次，或右键 **生成 builder.params**。
- 扩展目录中 `cl.eide-*` 的版本号随 EIDE 安装版本变化；也可在 EIDE 设置中查看 `EIDE_BUILDER_DIR` 实际路径。
- VS Code 用户将 `.cursor` 换为 `.vscode` 即可。

其他 unify_builder 参数：

| 参数 | 作用 |
|------|------|
| `--only-dump-args` | 仅打印编译器命令行，不实际编译 |
| `--only-dump-compilerdb` | 仅生成 `compile_commands.json` |
| `--dry-run` | 演练模式，不执行真实编译 |

### 4.3 查看完整编译命令

单文件完整编译命令记录在：

```text
build/M0G3519_nortos_keil/compile_commands.json
```

也可在 EIDE 项目右键菜单选择 **查看生成的编译器命令行**。

底层编译器为 Keil **ARMCLANG 6**（`armclang.exe`、`armasm.exe`、`armlink.exe`）。

### 4.4 输出目录

```text
M0G3519/keil/build/M0G3519_nortos_keil/
```

主要产物：

| 文件 | 说明 |
|------|------|
| `M0G3519_nortos_keil.axf` | 调试 / 链接输出 |
| `M0G3519_nortos_keil.hex` | 烧录用固件 |
| `compile_commands.json` | clangd / IDE 索引用 |
| `builder.params` | unify_builder 构建参数 |

---

## 5. 常见问题

### `'$P..' 不是内部或外部命令`

**原因**：预构建任务使用了 Keil uVision 专用变量 `$P`，EIDE 不会展开。  
**处理**：确认 `beforeBuildTasks` 使用 `${projectRoot}`（见 2.2）。

### `Undefined symbol DL_Common_delayCycles`

**原因**：未链接 TI DriverLib 预编译库。  
**处理**：在 `linker.misc-controls` 中添加 `driverlib.a`（路径 `mspm0gx51x`）。

### `L6242E: wchart-16 clashes with wchart-32`

**原因**：工程编译选项与 `driverlib.a` 的 ABI 不一致。  
**处理**：设置 `short-enums#wchar: true`，然后 **Clean + Rebuild**。

### SysConfig 找不到 SDK

**原因**：`syscfg.bat` 中 `SDK_ROOT` 不正确。  
**处理**：确认 `<SDK_ROOT>/.metadata/product.json` 存在。

### 头文件找不到（如 `ti/driverlib/...`）

**原因**：`incList` 未指向 SDK 的 `source` 目录。  
**处理**：检查 2.3 节路径配置。

---

## 6. 配置文件一览

| 文件 | 作用 |
|------|------|
| `M0G3519_nortos_keil.code-workspace` | VS Code / Cursor 工作区入口 |
| `.eide/eide.yml` | EIDE 工程主配置 |
| `.eide/env.ini` | 本机环境变量（可选） |
| `syscfg.bat` | 预构建：调用 SysConfig |
| `mspm0g3519.sct` | 链接 Scatter 文件 |
| `startup_mspm0g351x_uvision.s` | 启动汇编 |
| `ti_msp_dl_config.c/h` | SysConfig 生成（构建前自动更新） |
| `../M0G3519.syscfg` | SysConfig 硬件配置 |
| `../src/main.c` | 应用入口 |
| `build/M0G3519_nortos_keil/builder.params` | unify_builder 构建参数（构建时生成） |

---

## 7. 更换 SDK 版本时

若升级 MSPM0 SDK，请同步检查：

1. `syscfg.bat` 中的 `SDK_ROOT`
2. `eide.yml` 中所有 SDK 相关路径
3. SysConfig 版本是否与 SDK 发行说明匹配（必要时更新 `SYSCFG_CLI`）
4. `driverlib.a` 路径是否仍位于 `source/ti/driverlib/lib/keil/m0p/mspm0gx51x/`

修改后务必 **Clean + Rebuild** 验证。
