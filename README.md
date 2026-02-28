# Team 11319 - 2026 Season Robot Code

![FRC](https://img.shields.io/badge/FRC-2026-blue)
![Java](https://img.shields.io/badge/Java-17+-green)
![WPILib](https://img.shields.io/badge/WPILib-2025-orange)

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
- **框架**: WPILib 2025
- **架构**: AdvantageKit
- **硬件**:
  - CTRE TalonFX (驱动电机)
  - REV Spark MAX (辅助电机)
  - PhotonVision (视觉)
  - NavX / Pigeon 2 (陀螺仪)

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

- [WPILib](https://github.com/wpilibsuite/allwpilib)
- [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit)
- [CTRE Phoenix](https://github.com/CrossTheRoadElec/Phoenix6)
- [REV Lib](https://github.com/REVrobotics/REV-Software-Binary)
- [PhotonVision](https://github.com/PhotonVision/photonvision)
