# ArmDemo

## English

### Overview

ArmDemo is a WPILib 2026 Java project for team 11019 X.PLORE. It controls a dual-motor arm and a dual-motor elevator with Kraken X60 motors, TalonFX integrated encoders, software travel limits, AdvantageKit logging, and AdvantageScope telemetry.

The arm voltage can be tuned through NetworkTables. The elevator uses a fixed voltage. A single controller button press starts each movement command, and the command continues until the corresponding software limit is reached.

### Safety Warning

> **Before every deployment or robot-code restart, manually place both the arm and elevator at their physical zero positions.** The program resets all four TalonFX integrated encoders to zero during startup. Starting the robot away from the physical zero positions will make the software limits inaccurate and may damage the mechanisms.

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
| 4 | Raise arm | Runs until the arm reaches 90 degrees |
| 2 | Lower arm | Runs until the arm reaches 0 degrees |
| 3 | Extend elevator | Runs until either elevator motor reaches its own upper limit |
| 1 | Retract elevator | Runs until either elevator motor reaches the zero-position lower limit |

Each binding uses `onTrue`, so one press starts the command and the button does not need to remain held.

### Arm Control and Limits

- Motors: CAN IDs 1 and 2
- Encoder measurement: average of both TalonFX integrated encoder positions
- Minimum angle: `0°`
- Maximum angle: `90°`
- Calibrated travel from 0 to 90 degrees: `0.2` average motor rotations
- Default voltage: `3V`
- Maximum allowed voltage: `6V`
- NetworkTables tuning entry: `/SmartDashboard/Arm Voltage`

The requested dashboard voltage is treated as a magnitude, checked for a finite value, and limited to `6V`. The software applies the appropriate sign for upward or downward movement and stops the arm at either angular limit.

### Elevator Control and Limits

- Motors: CAN IDs 11 and 12
- Fixed movement voltage magnitude: `3V`
- Extension output: motor 1 receives `+3V`; motor 2 receives `-3V`
- Retraction output: motor 1 receives `-3V`; motor 2 receives `+3V`
- Lower limit: `0.0` rotations for both normalized encoder positions
- Motor 1 upper limit: `3.4` normalized rotations
- Motor 2 upper limit: `14.5` normalized rotations

The elevator encoders are not averaged because the two mechanisms have different travel ratios. Each encoder is normalized independently so extension is positive. If either motor reaches its own upper or lower limit, both motors stop.

### AdvantageScope Telemetry

Arm values are published under `RealOutputs/Arm`, including:

- `AngleDegrees`
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

### Command Structure

The movement commands are small command factories inside `ArmSubsystem` and `ElevatorSubsystem`. They use WPILib `Commands.runEnd(...)`, stop when a limit condition becomes true, and stop the motors when the command finishes or is interrupted. Because no standalone command classes are currently required, a separate `commands` package is not used.

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
    ├── ArmIOTalonFX.java
    ├── ArmSubsystem.java
    ├── ElevatorIOTalonFX.java
    └── ElevatorSubsystem.java
```

Subsystem tests are located in `src/test/java/frc/robot/subsystems/`. Daily bilingual engineering logs are located in `工程日志/`.

## 中文

### 项目简介

ArmDemo 是 11019 X.PLORE 的 WPILib 2026 Java 项目。项目使用 Kraken X60 电机和 TalonFX 内置编码器控制双电机机械臂与双电机 Elevator，并包含软件行程限位、AdvantageKit 日志和 AdvantageScope 遥测。

机械臂电压可以通过 NetworkTables 调节，Elevator 使用固定电压。每个动作只需按一次手柄按键，命令会持续运行，直到到达相应的软件限位。

### 安全警告

> **每次部署代码或重启机器人程序前，必须人工将机械臂和 Elevator 放到物理零位。** 程序会在启动时将四个 TalonFX 内置编码器全部归零。如果机器人启动时机构不在物理零位，软件限位将不准确，并可能损坏机构。

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
| 4 | 机械臂上升 | 持续运行，直到机械臂到达 90 度 |
| 2 | 机械臂下降 | 持续运行，直到机械臂到达 0 度 |
| 3 | Elevator 伸出 | 持续运行，直到任一 Elevator 电机到达自己的上限 |
| 1 | Elevator 收回 | 持续运行，直到任一 Elevator 电机到达零位下限 |

所有按键都使用 `onTrue` 绑定，因此按一次即可启动命令，不需要一直按住。

### 机械臂控制与限位

- 电机 CAN ID：1 和 2
- 编码器测量：两个 TalonFX 内置编码器位置的平均值
- 最小角度：`0°`
- 最大角度：`90°`
- 从 0 到 90 度的标定行程：平均 `0.2` 电机圈数
- 默认电压：`3V`
- 最大允许电压：`6V`
- NetworkTables 调节路径：`/SmartDashboard/Arm Voltage`

程序将面板输入电压作为绝对值使用，检查它是否为有限数值，并限制在 `6V` 以内。程序会根据机械臂上升或下降自动设置电压符号，并在任一角度限位处停止。

### Elevator 控制与限位

- 电机 CAN ID：11 和 12
- 固定运动电压：`3V`
- 伸出输出：电机 1 为 `+3V`，电机 2 为 `-3V`
- 收回输出：电机 1 为 `-3V`，电机 2 为 `+3V`
- 下限：两个方向修正后的编码器位置均以 `0.0` 圈为零位
- 电机 1 上限：`3.4` 圈
- 电机 2 上限：`14.5` 圈

由于两个机构的行程比例不同，程序不会平均 Elevator 的两个编码器。两个编码器分别进行方向修正，使伸出方向显示为正值。任一电机到达自己的上限或下限时，两台电机都会停止。

### AdvantageScope 遥测

机械臂数据发布在 `RealOutputs/Arm` 下，包括：

- `AngleDegrees`
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

### Command 结构

运动命令通过 `ArmSubsystem` 和 `ElevatorSubsystem` 内的小型命令工厂创建。它们使用 WPILib 的 `Commands.runEnd(...)`，在限位条件成立时结束，并在命令正常结束或被中断时停止电机。当前不需要独立的 Command 类，因此项目没有使用单独的 `commands` 包。

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
    ├── ArmIOTalonFX.java
    ├── ArmSubsystem.java
    ├── ElevatorIOTalonFX.java
    └── ElevatorSubsystem.java
```

子系统测试位于 `src/test/java/frc/robot/subsystems/`，每日双语工程日志位于 `工程日志/`。
