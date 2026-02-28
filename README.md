# Team 11319 - 2026 Season Robot Code

![FRC](https://img.shields.io/badge/FRC-2026-blue)
![Java](https://img.shields.io/badge/Java-17+-green)
![WPILib](https://img.shields.io/badge/WPILib-2026-orange)

FRC Team 11319 的 2026 赛季机器人代码，使用 AdvantageKit 架构。

## 项目结构

```
src/main/java/frc/robot/
├── Robot.java                 # 机器人主程序入口
├── RobotContainer.java        # 机器人组件容器
├── Constants.java             # 常量配置
├── commands/                  # 命令目录
│   ├── BLineCommands.java
│   └── DriveCommands.java
├── subsystems/                # 子系统目录
│   ├── bline/               # B-Line 路径跟踪
│   ├── drive/               # 底盘驱动 (Swerve)
│   ├── feeder/              # 供料系统
│   ├── intake/              #  intakes
│   ├── shooter/             # 射击系统 (Flywheel, Hood, Turret)
│   └── vision/              # 视觉系统 (PhotonVision)
└── util/                     # 工具类
```

## 技术栈

- **编程语言**: Java 17
- **框架**: WPILib 2026
- **架构**: AdvantageKit
- **软件库**:
  - YAMS — 机构控制库 (电梯、机械臂、炮塔等)
- **电机控制器**:
  - CTRE TalonFX (驱动电机)
  - REV Spark MAX (辅助电机)
- **传感器**:
  - CTRE CANcoder (编码器)
  - CTRE Pigeon 2 (陀螺仪)
- **视觉**:
  - PhotonVision (视觉)

## 构建与部署

### 前提条件

- JDK 17 或更高版本
- Gradle (通过 gradlew 自动下载)

### 构建命令

```bash
# 构建项目
./gradlew build

# 部署到机器人
./gradlew deploy

# 运行模拟器
./gradlew simulate
```

## 许可证

本项目基于 BSD 3-Clause License 开源。详情请参阅 [LICENSE](LICENSE) 文件。

## 致谢

本项目基于以下开源库构建：

- **[WPILib](https://github.com/wpilibsuite/allwpilib)** — FRC 官方机器人类库
- **[AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit)** — 机器人代码架构和数据记录
- **[CTRE Phoenix 6](https://github.com/CrossTheRoadElec/Phoenix6)** — TalonFX 电机驱动库
- **[REV Lib](https://github.com/REVrobotics/REV-Software-Binary)** — Spark MAX 电机驱动库
- **[PhotonVision](https://github.com/PhotonVision/photonvision)** — 视觉处理系统
- **[BLine-Lib](https://github.com/edanliahovetsky/BLine-Lib)** — 路径跟踪库
- **[YAMS](https://github.com/Yet-Another-Software-Suite/YAMS)** — 机构控制库 (LGPL v3.0)

## 基于模板

本项目基于 Littleton Robotics (FRC 6328 "Mechanical Advantage") 的 AdvantageKit TalonFX Swerve 模板开发。

---

Copyright (c) 2026 Team 11319. All rights reserved.
