# MSPM0G3519 Empty 工程 — EIDE 配置说明

本文档说明：拿到本空项目后，如何在 **VS Code + EIDE** 环境下完成配置并成功编译。

目标芯片：**MSPM0G3519**（Cortex-M0+）  
工程目录：`empty/keil/`  
应用源码：`empty/`（`empty.c`、`empty.syscfg`）  
SysConfig 生成文件：`empty/keil/`（`ti_msp_dl_config.c/h`）

---

## 1. 环境准备

请在本机安装以下工具，并确认路径可用：

| 工具 | 用途 | 示例路径 |
|------|------|----------|
| [EIDE 扩展](https://marketplace.visualstudio.com/items?itemName=cl.eide) | 嵌入式构建 | VS Code 扩展 |
| Keil MDK (ARM Compiler 6) | 编译 / 链接 | `A:/Program Files (x86)/Keil_v5/ARM/ARMCLANG` |
| MSPM0 SDK | 头文件、DriverLib、SysConfig 元数据 | `A:/ti/mspm0_sdk_2_10_00_04` |
| SysConfig | 根据 `.syscfg` 生成 `ti_msp_dl_config.*` | `A:/ti/sysconfig_1.28.0/sysconfig_cli.bat` |

打开工程：

```text
empty/keil/empty_LP_MSPM0G3519_nortos_keil.code-workspace
```

在 EIDE 中配置 Keil 工具链路径：**Settings → EIDE → ARM.AC6 → Keil 安装目录**。

---

## 2. 必须修改的配置（按顺序）

以下路径请替换为你本机实际安装位置。下文用占位符表示：

- `<SDK_ROOT>` — MSPM0 SDK 根目录，例如 `A:/ti/mspm0_sdk_2_10_00_04`
- `<SYSCONFIG_CLI>` — SysConfig 命令行入口，例如 `A:/ti/sysconfig_1.28.0/sysconfig_cli.bat`

Windows 路径在 `eide.yml` 中建议使用 **正斜杠** `/`，避免转义问题。

---

### 步骤 1：修改 `syscfg.bat`

文件：`empty/keil/syscfg.bat`

构建前会运行此脚本，调用 SysConfig 生成 `ti_msp_dl_config.c` / `ti_msp_dl_config.h`。

修改两处：

```bat
set SYSCFG_PATH="<SYSCONFIG_CLI>"
set SDK_ROOT=<SDK_ROOT>
```

说明：

- `SYSCFG_PATH`：指向 `sysconfig_cli.bat`。
- `SDK_ROOT`：指向 MSPM0 SDK 根目录（需包含 `.metadata/product.json`）。
- 不要使用 Keil 专用的 `$P` 变量；EIDE 不会展开它。
- **生成文件输出目录**：SysConfig 的 `-o` 参数决定 `ti_msp_dl_config.*` 写到哪里。本工程使用 `-o "%PROJ_DIR%"`，即输出到 `empty/keil/`（与 EIDE 的 `${projectRoot}` 一致），而 `.syscfg` 仍可从上级目录 `empty/` 读取。

---

### 步骤 2：修改 `.eide/eide.yml`

文件：`empty/keil/.eide/eide.yml`

#### 2.1 预构建任务（SysConfig）

确认 `beforeBuildTasks` 使用 EIDE 变量 `${projectRoot}`，**不要**使用 Keil 的 `$P`：

```yaml
beforeBuildTasks:
  - name: linking syscfg
    abortAfterFailed: true
    command: '"${projectRoot}/syscfg.bat" "${projectRoot}" empty.syscfg'
    disable: false
    stopBuildAfterFailed: true
```

#### 2.2 头文件搜索路径

将 SDK 路径写入 `incList`（替换 `<SDK_ROOT>`）：

```yaml
incList:
  - .
  - ..
  - <SDK_ROOT>/source
  - <SDK_ROOT>/source/third_party/CMSIS/Core/Include
  - .cmsis/include
  - RTE/_empty_LP_MSPM0G3519_nortos_keil
```

- `.` 对应 `empty/keil/`，用于 `ti_msp_dl_config.h` 等 SysConfig 生成文件。
- `..` 对应 `empty/`，供 `empty.c` 等同目录源码使用。
- 不要继续使用原 Keil 工程里向上多级跳转的相对路径（如 `../../../../../../source`），在本仓库结构下会指向错误目录。

#### 2.3 CPU 型号

MSPM0G3519 为 **Cortex-M0+**：

```yaml
cpuType: Cortex-M0+
```

#### 2.4 编译选项（与 DriverLib 库 ABI 一致）

链接 SDK 预编译的 `driverlib.a` 时，必须开启短 enum / 短 wchar，否则会报 `L6242E`：

```yaml
c/cpp-compiler:
  short-enums#wchar: true
```

等价于编译器参数：`-fshort-enums -fshort-wchar`。

#### 2.5 链接 DriverLib

在 `linker.misc-controls` 中加入 SDK 中的 Keil 库（替换 `<SDK_ROOT>`）：

```yaml
linker:
  misc-controls: <SDK_ROOT>/source/ti/driverlib/lib/keil/m0p/mspm0gx51x/driverlib.a --diag_suppress=L6329
  output-format: elf
```

#### 2.7 SysConfig 生成文件路径（`eide.yml` 源文件列表）

若希望 `ti_msp_dl_config.c/h` 位于 `keil/` 而非 `empty/`，需同时满足：

1. `syscfg.bat` 最后一行使用 `-o "%PROJ_DIR%"`（输出到工程根目录 `keil/`）。
2. `eide.yml` 中源文件引用改为本地路径：

```yaml
- path: ./ti_msp_dl_config.h
- path: ./ti_msp_dl_config.c
```

3. `incList` 包含 `.`（见 2.2），以便编译 `../empty.c` 时能找到 `keil/` 下的头文件。

`.syscfg` 仍可放在 `empty/empty.syscfg`，无需移动。

---

#### 2.8 Scatter 文件

使用工程自带的链接脚本：

```yaml
scatterFilePath: ./mspm0g3519.sct
useCustomScatterFile: true
```

---

## 3. 构建步骤

1. 用 VS Code 打开 `empty_LP_MSPM0G3519_nortos_keil.code-workspace`。
2. 按上文完成 `syscfg.bat` 与 `eide.yml` 修改。
3. 在 EIDE 侧边栏选择目标 **empty_LP_MSPM0G3519_nortos_keil**。
4. 首次或修改编译选项后：**Clean → Rebuild**（必须全量重编，否则旧的 `.o` 可能仍带错误 ABI）。
5. 正常时日志顺序为：
   - `pre-build tasks` → SysConfig 成功
   - `compiling` → 编译 `.c` / 汇编
   - `linking` → 生成 `.axf`

输出目录：

```text
empty/keil/build/empty_LP_MSPM0G3519_nortos_keil/
```

---

## 4. 常见问题

### `'$P..' 不是内部或外部命令`

**原因**：预构建任务使用了 Keil uVision 专用变量 `$P`，EIDE 不会展开。  
**处理**：按步骤 2.1 改为 `${projectRoot}`。

### `Undefined symbol DL_Common_delayCycles`

**原因**：未链接 TI DriverLib 预编译库。  
**处理**：按步骤 2.5 添加 `driverlib.a`（路径为 `mspm0gx51x`）。

### `L6242E: wchart-16 clashes with wchart-32` / `packed-enum clashes with enum_is_int`

**原因**：工程编译选项与 `driverlib.a` 的 ABI 不一致。  
**处理**：按步骤 2.4 设置 `short-enums#wchar: true`，然后 **Clean + Rebuild**。

### SysConfig 找不到 SDK

**原因**：`syscfg.bat` 中 `SDK_ROOT` 不正确。  
**处理**：确认 `<SDK_ROOT>/.metadata/product.json` 存在。

### 头文件找不到（如 `ti/driverlib/...`）

**原因**：`incList` 未指向 SDK 的 `source` 目录，或路径层级错误。  
**处理**：按步骤 2.2 检查路径。

---

## 5. 配置文件一览

| 文件 | 作用 |
|------|------|
| `syscfg.bat` | 调用 SysConfig；需配置 SDK 与 SysConfig 路径；`-o` 指向 `keil/` |
| `.eide/eide.yml` | EIDE 工程主配置（包含路径、编译、链接、预构建任务） |
| `mspm0g3519.sct` | 链接 Scatter 文件 |
| `ti_msp_dl_config.c/h` | SysConfig 生成（位于 `keil/`，构建前自动更新） |
| `../empty.syscfg` | SysConfig 硬件配置文件 |
| `../empty.c` | 应用入口 |

---

## 6. 更换 SDK 版本时

若升级 MSPM0 SDK，请同步检查：

1. `syscfg.bat` 中的 `SDK_ROOT`
2. `eide.yml` 中所有 `<SDK_ROOT>` 路径
3. SysConfig 版本是否与 SDK 发行说明匹配（必要时更新 `SYSCFG_PATH`）
4. `driverlib.a` 路径是否仍位于 `source/ti/driverlib/lib/keil/m0p/mspm0gx51x/`

修改后务必 **Clean + Rebuild** 验证。
