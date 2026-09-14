# ArmDemo

## English

### Overview

ArmDemo is a WPILib 2026 Java project for team 11019 X.PLORE. It controls a dual-motor arm and a dual-motor elevator with Kraken X60 motors, TalonFX integrated encoders, software travel limits, AdvantageKit logging, and AdvantageScope telemetry.

The arm voltage can be tuned through NetworkTables. The elevator uses a fixed voltage. A single controller button press starts each movement command, and the command continues until the corresponding software limit is reached. After a movement command finishes, the arm actively holds its angle — a continuous gravity-compensation voltage with a fade near the target at the top, a gentle lean on the software zero at the bottom — so it no longer falls under gravity.

### Safety Warning

> **Before every roboRIO power-on, manually place both the arm and elevator at their physical zero positions.** After a full power cycle the TalonFX integrated encoders read zero on their own, so mechanisms that are not at their physical zero positions at power-on will make the software limits inaccurate and may damage the mechanisms.
>
> Robot-code restarts (redeploy or soft restart) do **not** re-zero the encoders: the TalonFX positions survive a code restart, so the software limits stay accurate even when the code is restarted mid-match with the mechanisms away from zero.

Keep the robot disabled while positioning the mechanisms by hand. Stay clear of the arm and elevator whenever the robot is enabled.

### Hardware Configuration

| Mechanism | Motor | CAN ID | Neutral Mode | Encoder Direction During Positive Extension |
| --- | --- | ---: | --- | --- |
| Arm | Kraken X60 / TalonFX 1 | 1 | Brake | Positive |
| Arm | Kraken X60 / TalonFX 2 | 2 | Brake | Positive |
| Elevator | Kraken X60 / TalonFX 1 | 11 | Brake | Raw value decreases; software negates it |
| Elevator | Kraken X60 / TalonFX 2 | 12 | Brake | Raw value decreases; software negates it |

The driver controller is connected to USB port 0. Although the code uses `CommandPS5Controller`, this controller reports Xbox-style raw button IDs in Driver Station.

### Controller Bindings

| Raw Button | Action | Command Behavior |
| ---: | --- | --- |
| 4 | Raise arm | Moves smoothly to the configurable target angle (default 90°) and keeps holding there |
| 2 | Lower arm | Moves smoothly to 0 degrees and keeps holding there |
| 5 | Arm to middle | Moves smoothly to the middle working angle (default 45°) from whichever side the arm is on, then keeps holding there |
| 3 | Extend elevator | Runs until either elevator motor reaches its own upper limit; only allowed after the arm has reached its target angle (default 90°) |
| 1 | Retract elevator | Runs until either elevator motor reaches the zero-position lower limit; only allowed after the arm has reached its target angle (default 90°) |
| 6 | Elevator to middle | Runs until either elevator motor reaches its own share of the middle working extension (default half travel), from whichever side it is on; only allowed after the arm has reached its target angle |

Each binding uses `onTrue`, so one press starts the command and the button does not need to remain held. Whenever no movement command is running, a default hold command keeps the arm at its latched angle. Every movement command also records how long it ran; the duration is printed to the robot console and published as telemetry (`LastMoveDurationSeconds` / `LastMoveActionName`) for the spec's per-action timing records.

### Arm Control and Limits

- Motors: CAN IDs 1 and 2
- Encoder measurement: average of both TalonFX integrated encoder positions
- Minimum angle: `0°`
- Maximum angle: `90°`
- Middle working angle (button 5): `45°`, tolerance window `1.5°`
- Calibrated travel from 0 to 90 degrees: `0.2` average motor rotations
- Default voltage: `3V`
- Maximum allowed voltage: `3V`
- NetworkTables tuning entry: `/SmartDashboard/Arm Voltage`

The requested dashboard voltage is treated as a magnitude, checked for a finite value, and limited to `3V`. The software applies the appropriate sign for upward or downward movement and stops the arm at either angular limit.

### Arm Motion and Position Hold

Motion runs open-loop on voltage, exactly like the original code the team validated as smooth: mid-travel the arm receives the full requested voltage. Rising tapers linearly to a `0.8V` floor at the target and falling tapers all the way to `0V` at the lower limit (gravity finishes the descent), so the arm always arrives gently.

Button 4 raises the arm to the configurable target angle `TARGET_ANGLE_DEGREES` (default `90°`, spec allows `85°`); the hard software limit stays at `90°` regardless.

After a movement command finishes (or is interrupted), the default `ArmHoldPosition` command latches the current angle and holds it in one of three zones:

- **Upper zone** (latched within `15°` of the target): a continuous gravity-compensation voltage (`HOLD_VOLTAGE`, default `2.4V`, tunable at `/SmartDashboard/Arm Hold Voltage`) that fades linearly to zero over the last `2°` below the target, using a lightly filtered angle so encoder quantization cannot hunt. The arm lands on the target with zero closing speed and never crosses the limit wall.
- **Bottom zone** (latched within `1°` of zero): the arm leans on the software zero from below with `0.6V`, fading over the last degree — the mirror of the top fade.
- **Middle**: a one-way hysteresis nudge at `1.6V` — sag more than `1°`, push back, release within `0.5°`; strictly one-directional, so it cannot oscillate.

The controller never commands a setpoint outside `0–90°`, the output never exceeds `±3V`, and the software limit wall blocks any output that would push the arm further past a limit it is already at. When the arm is not calibrated, the hold outputs `0V`.

- NetworkTables tuning entries: `/SmartDashboard/Arm Voltage` (movement magnitude), `/SmartDashboard/Arm Hold Voltage` (top hold strength)

**Through-bore encoder switch**: `USE_THROUGH_BORE_ENCODER` (default `false`) selects between the averaged TalonFX rotor positions and the absolute shaft encoder (`ArmIOThroughBore`). The DIO channel, gear ratio, and zero-offset constants are placeholders to fill in when the encoders arrive; switching removes the startup zeroing requirement.

### Speed and Acceleration Limits

Both mechanisms keep their validated open-loop voltage motion; the spec's "settable max speed / max acceleration" is layered on top as live-tunable safety ceilings:

- **Acceleration limit**: movement voltages ramp at most a configurable number of volts per second (`Arm Voltage Slew (V/s)` / `Elevator Voltage Slew (V/s)`, default `60 V/s` — full voltage within about three 20 ms cycles, i.e. effectively the validated behavior). The ramp applies to the signed voltage, so a direction reversal passes through zero volts instead of snapping to the opposite polarity. The hold outputs are not ramped: they are already gentle by design, and the stop must stay instant.
- **Speed limit**: when the measured motion already runs at the cap in the commanded direction, the movement voltage is cut to zero. The arm uses its filtered angle velocity against `Arm Max Speed (deg/s)` (default `60°/s`); the elevator compares each motor's extension velocity — normalized by its own travel so the two mechanisms stay coordinated — against `Elevator Max Travel (/s)` (default `1.5` full travels per second).
- Setting any entry to `0` disables that limit; a mistyped (non-numeric or negative) entry falls back to its validated default instead of silently disabling the limit. All four entries are on NetworkTables for the tuning weeks.

### Action Timing

Every movement command (arm up/down/middle, elevator extend/retract/middle) measures its own runtime, prints `Arm action <name> took <t> s` (or the elevator equivalent) to the robot console, and publishes `LastMoveDurationSeconds` plus `LastMoveActionName` under `RealOutputs/Arm` / `RealOutputs/Elevator` — the numbers for the spec's per-action completion-time records.

### Elevator Control and Limits

- Motors: CAN IDs 11 and 12
- Fixed movement voltage magnitude: `2V`
- Extension output: motor 1 receives `+2V`; motor 2 receives `-2V`
- Retraction output: motor 1 receives `-2V`; motor 2 receives `+2V`
- Lower limit: `0.0` rotations for both normalized encoder positions
- Motor 1 upper limit: `3.4` normalized rotations
- Motor 2 upper limit: `14.5` normalized rotations
- Middle working extension (button 6): half of each motor's own travel (1.7 / 7.25 rotations), so the two mechanisms arrive together

The elevator encoders are not averaged because the two mechanisms have different travel ratios. Each encoder is normalized independently so extension is positive. If either motor reaches its own upper or lower limit, both motors stop.

**Arm interlock**: elevator commands are only scheduled after the arm has reached its target angle, and the permission is re-checked every cycle while the elevator moves — if the arm drops below the unlock margin mid-motion (e.g. the arm-down button is pressed, or the hold fails), the elevator command ends immediately and both motors stop. The permission stays active while the arm rests near the top (the fade hold settles slightly below the target) and clears once the arm drops 10 degrees below it.

### AdvantageScope Telemetry

Arm values are published under `RealOutputs/Arm`, including:

- `AngleDegrees`
- `AngleVelocityDegreesPerSecond`
- `HoldSetpointDegrees`
- `HoldFilteredAngleDegrees`
- `Motor1Rotations`
- `Motor2Rotations`
- `AverageMotorRotations`
- `EncoderDifferenceRotations`
- `Calibrated`
- `AtLowerLimit`
- `AtUpperLimit`
- `RequestedVoltage`
- `AppliedVoltage`
- `MotorRotationsAtMaxAngle`
- `LastMoveDurationSeconds`
- `LastMoveActionName`

Elevator values are published under `RealOutputs/Elevator`, including:

- `Motor1ExtensionRotations`
- `Motor2ExtensionRotations`
- `RawMotor1Rotations`
- `RawMotor2Rotations`
- `Motor1AppliedVoltage`
- `Motor2AppliedVoltage`
- `AtLowerLimit`
- `AtUpperLimit`
- `Motor1MaxExtensionRotations`
- `Motor2MaxExtensionRotations`
- `Motor1TravelFractionPerSecond`
- `Motor2TravelFractionPerSecond`
- `LastMoveDurationSeconds`
- `LastMoveActionName`

### Command Structure

The movement commands are small command factories inside `ArmSubsystem` and `ElevatorSubsystem`. They use WPILib `Commands.runEnd(...)`, stop when a limit condition becomes true, and stop the motors when the command finishes or is interrupted. `ArmSubsystem` also provides `holdCurrentPositionCommand()` (built with `Commands.startRun(...)`), which is registered as the arm's default command so the arm is always actively held while no movement command runs. Because no standalone command classes are currently required, a separate `commands` package is not used.

### Data Flow (AdvantageKit IO Pattern)

Each mechanism follows one data-flow pattern: **hardware → IO → subsystem → hardware**.

- `ArmIO` / `ElevatorIO` (top-level interfaces) define the hardware contract: an `updateInputs(inputs)` poll that fills an `@AutoLog` inputs container once per cycle, plus the output setters (`setVoltage`, `zeroEncoders`).
- `ArmIOTalonFX` / `ElevatorIOTalonFX` are the real-hardware implementations — all Phoenix 6 details live there; `ArmIOThroughBore` is the drop-in absolute-encoder variant.
- Each subsystem's `periodic()` runs exactly once per 20 ms loop and does three things in order: **poll** the NetworkTables tuning entries (sanitized once, shared by the whole cycle), **poll** the sensors via `io.updateInputs(inputs)` and log them with `Logger.processInputs`, then run the control logic against those snapshots.
- `RobotContainer` is wiring only: which IO implementation backs each subsystem, the arm→elevator interlock connection, and the button bindings. It holds no tuning data.
- Test doubles (`FakeArmIO`, `FakeElevatorIO`) implement the same interfaces with plain fields, so all subsystem logic is tested without hardware.

### Build, Test, and Deploy

Requirements:

- WPILib 2026
- Java 17 from the WPILib 2026 installation
- Phoenix 6 and AdvantageKit vendor dependencies included in `vendordeps`

Build the project:

```sh
./gradlew build
```

Run the automated tests:

```sh
./gradlew test
```

Deploy with the WPILib VS Code command **WPILib: Deploy Robot Code**, or run:

```sh
./gradlew deploy
```

### Main Project Structure

```text
src/main/java/frc/robot/
├── Constants.java
├── Main.java
├── Robot.java
├── RobotContainer.java
└── subsystems/
    ├── ArmIO.java
    ├── ArmIOThroughBore.java
    ├── ArmIOTalonFX.java
    ├── ArmSubsystem.java
    ├── ElevatorIO.java
    ├── ElevatorIOTalonFX.java
    └── ElevatorSubsystem.java
```

Subsystem tests are located in `src/test/java/frc/robot/subsystems/`. Daily bilingual engineering logs are located in `工程日志/`.

## 中文

### 项目简介

ArmDemo 是 11019 X.PLORE 的 WPILib 2026 Java 项目。项目使用 Kraken X60 电机和 TalonFX 内置编码器控制双电机机械臂与双电机 Elevator，并包含软件行程限位、AdvantageKit 日志和 AdvantageScope 遥测。

机械臂电压可以通过 NetworkTables 调节，Elevator 使用固定电压。每个动作只需按一次手柄按键，命令会持续运行，直到到达相应的软件限位。运动命令结束后，机械臂会主动保持当前角度——顶部用带渐隐的持续重力补偿电压、底部轻倚软件零位——不会再因重力下坠。

### 安全警告

> **每次 roboRIO 上电前，必须人工将机械臂和 Elevator 放到物理零位。** 完全断电重启后 TalonFX 内置编码器会自行从零开始读数，如果上电时机构不在物理零位，软件限位将不准确，并可能损坏机构。
>
> 重启机器人程序（重新部署或软重启）**不会**重新归零编码器：TalonFX 位置在代码重启后保持不变，因此比赛中途软重启时，即使机构不在零位，软件限位依然准确。

人工调整机构时必须保持机器人 Disabled。机器人 Enabled 后，所有人员都应远离机械臂和 Elevator 的运动范围。

### 硬件配置

| 机构 | 电机 | CAN ID | 停止模式 | 正向伸出时的编码器方向 |
| --- | --- | ---: | --- | --- |
| 机械臂 | Kraken X60 / TalonFX 1 | 1 | Brake | 正值 |
| 机械臂 | Kraken X60 / TalonFX 2 | 2 | Brake | 正值 |
| Elevator | Kraken X60 / TalonFX 1 | 11 | Brake | 原始值减小，软件中取反 |
| Elevator | Kraken X60 / TalonFX 2 | 12 | Brake | 原始值减小，软件中取反 |

驾驶员手柄连接到 USB 端口 0。虽然代码使用 `CommandPS5Controller`，但当前手柄在 Driver Station 中返回 Xbox 风格的原始按键编号。

### 手柄按键

| 原始按键 | 动作 | 命令行为 |
| ---: | --- | --- |
| 4 | 机械臂上升 | 平滑移动到可配置目标角（默认 90 度）并持续保持 |
| 2 | 机械臂下降 | 平滑移动到 0 度并持续保持 |
| 5 | 机械臂中位 | 从任意一侧平滑移动到中间工作角（默认 45 度）并持续保持 |
| 3 | Elevator 伸出 | 持续运行，直到任一 Elevator 电机到达自己的上限；仅当机械臂已到达目标角（默认 90 度）后才允许 |
| 1 | Elevator 收回 | 持续运行，直到任一 Elevator 电机到达零位下限；仅当机械臂已到达目标角（默认 90 度）后才允许 |
| 6 | Elevator 中位 | 从任意一侧运行，直到任一电机到达自己份额的中间工作行程（默认半行程）；仅当机械臂已到达目标角后才允许 |

所有按键都使用 `onTrue` 绑定，因此按一次即可启动命令，不需要一直按住。只要没有运动命令在运行，默认保持命令就会让机械臂锁定在当前角度。每个运动命令还会记录自己的运行时长：完成后在机器人控制台打印一行日志，并作为遥测量（`LastMoveDurationSeconds` / `LastMoveActionName`）发布——这就是规格里"每个动作记录完成时间"的数据来源。

### 机械臂控制与限位

- 电机 CAN ID：1 和 2
- 编码器测量：两个 TalonFX 内置编码器位置的平均值
- 最小角度：`0°`
- 最大角度：`90°`
- 中间工作角（按键 5）：`45°`，到位窗口 `1.5°`
- 从 0 到 90 度的标定行程：平均 `0.2` 电机圈数
- 默认电压：`3V`
- 最大允许电压：`3V`
- NetworkTables 调节路径：`/SmartDashboard/Arm Voltage`

程序将面板输入电压作为绝对值使用，检查它是否为有限数值，并限制在 `3V` 以内。程序会根据机械臂上升或下降自动设置电压符号，并在任一角度限位处停止。

### 机械臂运动与位置保持

运动采用开环电压控制，与队伍验证过平顺性的最初版本完全一致：行程中段输出全额请求电压。上升在目标角前线性减速至 `0.8V` 底值；下降在零位前一路归零（由重力完成末段），机械臂始终柔和到位。

按键 4 将机械臂升到可配置的目标角 `TARGET_ANGLE_DEGREES`（默认 `90°`，规格允许 `85°`）；无论目标设多少，硬软件限位始终是 `90°`。

任何运动命令结束（或被打断）后，默认的 `ArmHoldPosition` 命令锁存当前角度，并按三个区间保持：

- **顶部区间**（锁存点距目标 `15°` 以内）：持续重力补偿电压（`HOLD_VOLTAGE`，默认 `2.4V`，可在 `/SmartDashboard/Arm Hold Voltage` 在线调节），在目标下方最后 `2°` 内线性渐隐到零，并使用轻度滤波角度以避免编码器量化引起狩猎。机械臂以零闭合力速度落在目标上，永远不会越过限位墙。
- **底部区间**（锁存点距零位 `1°` 以内）：以 `0.6V` 从下方倚住软件零位，最后 `1°` 渐隐——与顶部渐隐完全镜像。
- **中段**：`1.6V` 单向迟滞轻推——下垂超过 `1°` 推回、回到 `0.5°` 内释放；严格单向，不可能振荡。

控制器给出的目标角度永远限制在 `0–90°` 以内，输出电压不超过 `±3V`，软件限位墙阻断任何把已越限机械臂往限位外推的输出。机械臂未标定时，保持输出 `0V`。

- NetworkTables 调节路径：`/SmartDashboard/Arm Voltage`（运动电压幅值）、`/SmartDashboard/Arm Hold Voltage`（顶部保持力度）

**Through-bore 编码器切换**：`USE_THROUGH_BORE_ENCODER`（默认 `false`）在"TalonFX 转子平均"与"轴端绝对编码器"（`ArmIOThroughBore`）之间切换。DIO 通道、速比、零位偏移常量为占位值，编码器到货后填入；切换后不再依赖启动归零。

### 速度与加速度限制

两套机构都保留验证过的开环电压运动；规格要求的"可设置并限制最大速度、最大加速度"以可在线调节的安全上限形式叠加在之上：

- **加速度限制**：运动电压每秒最多变化可配置的伏特数（`Arm Voltage Slew (V/s)` / `Elevator Voltage Slew (V/s)`，默认 `60 V/s`——约三个 20 ms 周期内到达满电压，即实际等同于已验证的行为）。斜坡作用于带符号的电压，因此换向时电压会经过零点渐变，不会瞬间翻转到反极性。保持命令的输出不做斜坡：它本身变化就很柔和，而且停止必须保持即时。
- **速度限制**：当测量到的运动在指令方向上已经达到上限时，运动电压立即归零。机械臂用滤波后的角速度对比 `Arm Max Speed (deg/s)`（默认 `60°/s`）；Elevator 把每个电机的伸出速度按各自行程归一化（保证两机构协同）后对比 `Elevator Max Travel (/s)`（默认每秒 1.5 倍全行程）。
- 任何一项设为 `0` 即关闭该限制；误输入的非数值或负值会回退到已验证的默认值，而不会静默关闭限制。四项都在 NetworkTables 上，供调参周使用。

### 动作计时

每个运动命令（机械臂升/降/中位、Elevator 伸/收/中位）测量自己的运行时长，完成后在机器人控制台打印 `Arm action <名称> took <秒> s`（Elevator 同理），并在 `RealOutputs/Arm` / `RealOutputs/Elevator` 下发布 `LastMoveDurationSeconds` 和 `LastMoveActionName`——对应规格里"每个动作均记录完成时间"的要求。

### Elevator 控制与限位

- 电机 CAN ID：11 和 12
- 固定运动电压：`2V`
- 伸出输出：电机 1 为 `+2V`，电机 2 为 `-2V`
- 收回输出：电机 1 为 `-2V`，电机 2 为 `+2V`
- 下限：两个方向修正后的编码器位置均以 `0.0` 圈为零位
- 电机 1 上限：`3.4` 圈
- 电机 2 上限：`14.5` 圈
- 中间工作行程（按键 6）：各电机自身行程的一半（1.7 / 7.25 圈），两机构同步到位

由于两个机构的行程比例不同，程序不会平均 Elevator 的两个编码器。两个编码器分别进行方向修正，使伸出方向显示为正值。任一电机到达自己的上限或下限时，两台电机都会停止。

**机械臂联锁**：Elevator 命令只有在机械臂到达过目标角（默认 90 度）之后才会被调度，并且电梯运动期间联锁**每个周期都会复核**——若机械臂在电梯运动中途掉到解锁余量以下（例如有人按了机械臂下降键、或保持失效），电梯命令立即结束、双电机停车。机械臂在顶部停留期间（渐隐保持会略低于目标角）联锁保持有效，一旦机械臂掉回目标角以下 `10 度` 即失效。

### AdvantageScope 遥测

机械臂数据发布在 `RealOutputs/Arm` 下，包括：

- `AngleDegrees`
- `AngleVelocityDegreesPerSecond`
- `HoldSetpointDegrees`
- `HoldFilteredAngleDegrees`
- `Motor1Rotations`
- `Motor2Rotations`
- `AverageMotorRotations`
- `EncoderDifferenceRotations`
- `Calibrated`
- `AtLowerLimit`
- `AtUpperLimit`
- `RequestedVoltage`
- `AppliedVoltage`
- `MotorRotationsAtMaxAngle`
- `LastMoveDurationSeconds`
- `LastMoveActionName`

Elevator 数据发布在 `RealOutputs/Elevator` 下，包括：

- `Motor1ExtensionRotations`
- `Motor2ExtensionRotations`
- `RawMotor1Rotations`
- `RawMotor2Rotations`
- `Motor1AppliedVoltage`
- `Motor2AppliedVoltage`
- `AtLowerLimit`
- `AtUpperLimit`
- `Motor1MaxExtensionRotations`
- `Motor2MaxExtensionRotations`
- `Motor1TravelFractionPerSecond`
- `Motor2TravelFractionPerSecond`
- `LastMoveDurationSeconds`
- `LastMoveActionName`

### Command 结构

运动命令通过 `ArmSubsystem` 和 `ElevatorSubsystem` 内的小型命令工厂创建。它们使用 WPILib 的 `Commands.runEnd(...)`，在限位条件成立时结束，并在命令正常结束或被中断时停止电机。`ArmSubsystem` 还提供 `holdCurrentPositionCommand()`（使用 `Commands.startRun(...)` 构建），并被注册为机械臂的默认命令，保证没有运动命令运行时机械臂始终被主动保持。当前不需要独立的 Command 类，因此项目没有使用单独的 `commands` 包。

### 数据流（AdvantageKit IO 模式）

每个机构都遵循同一条数据流：**硬件 → IO → 子系统 → 硬件**。

- `ArmIO` / `ElevatorIO`（顶层接口）定义硬件契约：一个每周期调用一次的 `updateInputs(inputs)` 轮询，把传感器值填进 `@AutoLog` 输入容器；加上输出方法（`setVoltage`、`zeroEncoders`）。
- `ArmIOTalonFX` / `ElevatorIOTalonFX` 是真机实现——所有 Phoenix 6 细节都锁在这两个文件里；`ArmIOThroughBore` 是即插即用的绝对编码器变体。
- 每个子系统的 `periodic()` 在 20 ms 循环里恰好执行一次，按固定顺序做三件事：**轮询** NetworkTables 调参项（每周期消毒一次、全周期共享同一份值），通过 `io.updateInputs(inputs)` **轮询**传感器并用 `Logger.processInputs` 整体记录，然后基于这些快照运行控制逻辑。
- `RobotContainer` 只负责接线：每个子系统用哪个 IO 实现、机械臂到 Elevator 的联锁连接、以及按键绑定。它不持有任何调参数据。
- 测试替身（`FakeArmIO`、`FakeElevatorIO`）用普通字段实现同样的接口，因此全部子系统逻辑都可以在没有硬件的情况下测试。

### 构建、测试与部署

环境要求：

- WPILib 2026
- WPILib 2026 自带的 Java 17
- `vendordeps` 中已经包含 Phoenix 6 和 AdvantageKit 依赖

构建项目：

```sh
./gradlew build
```

运行自动化测试：

```sh
./gradlew test
```

可以使用 VS Code 中的 **WPILib: Deploy Robot Code** 命令部署，也可以运行：

```sh
./gradlew deploy
```

### 主要项目结构

```text
src/main/java/frc/robot/
├── Constants.java
├── Main.java
├── Robot.java
├── RobotContainer.java
└── subsystems/
    ├── ArmIO.java
    ├── ArmIOThroughBore.java
    ├── ArmIOTalonFX.java
    ├── ArmSubsystem.java
    ├── ElevatorIO.java
    ├── ElevatorIOTalonFX.java
    └── ElevatorSubsystem.java
```

子系统测试位于 `src/test/java/frc/robot/subsystems/`，每日双语工程日志位于 `工程日志/`。
