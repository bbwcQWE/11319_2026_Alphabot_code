// Copyright (c) 2025-2026 11319 Polaris
// https://github.com/bbwcQWE
//
// Based on Littleton Robotics AdvantageKit TalonFX(S) Swerve Template
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.bline.BLinePathFollower;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
// import frc.robot.subsystems.feeder.FeederSubsystem;
// import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.vision.Vision;
// import frc.robot.subsystems.vision.VisionConstants;
// import frc.robot.subsystems.vision.VisionIOPhotonVision;
// import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, OI devices, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Vision vision;
  private final BLinePathFollower blinePathFollower;
  // private final FeederSubsystem feeder;
  // private final IntakeSubsystem intake;
  // private final ShooterSubsystem shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // Instantiate vision subsystem with PhotonVision cameras
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0),
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera1Name, VisionConstants.robotToCamera1));

        // Initialize feeder and intake subsystems
        // feeder = new FeederSubsystem();
        // intake = new IntakeSubsystem();

        // Initialize shooter subsystem
        // shooter = new ShooterSubsystem();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // Instantiate vision subsystem with simulated cameras
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera1Name, VisionConstants.robotToCamera1,
        // drive::getPose));

        // Initialize feeder and intake subsystems
        // feeder = new FeederSubsystem();
        // intake = new IntakeSubsystem();

        // Initialize shooter subsystem
        // shooter = new ShooterSubsystem();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // Disable vision in replay mode
        // vision = new Vision(drive::addVisionMeasurement);

        // Initialize feeder and intake subsystems
        // feeder = new FeederSubsystem();
        // intake = new IntakeSubsystem();

        // Initialize shooter subsystem
        // shooter = new ShooterSubsystem();
        break;
    }

    // Register vision subsystem to enable periodic() calls
    // vision.register();

    // Register feeder and intake subsystems
    // feeder.register();
    // intake.register();

    // Register shooter subsystem to enable periodic() calls
    // shooter.register();

    // Initialize BLine Path Follower subsystem
    blinePathFollower = new BLinePathFollower(drive);

    // Set up auto routines using PathPlanner
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // 添加PathPlanner"None"选项作为默认（将回退到BLine检查）
    autoChooser.addDefaultOption("None (Use BLine)", Commands.runOnce(drive::stop, drive));

    // 配置BLine路径选择器
    configureBLineAutoChooser();

    // 添加系统选择器（PathPlanner / BLine / None）
    configureSystemChooser();

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /** BLine路径选择器 - 在dashboard上显示 */
  private final SendableChooser<String> blinePathChooser = new SendableChooser<>();

  /** Configure BLine auto routines for the chooser */
  private void configureBLineAutoChooser() {
    // 添加默认选项
    blinePathChooser.setDefaultOption("None", "");

    // 尝试多种方式获取路径文件
    boolean foundPaths = false;

    // 方法1: 尝试从deploy目录读取 (运行时)
    String[] possiblePaths = {
      "/home/lvuser/deploy/autos/paths", // RoboRIO路径
      "src/main/deploy/autos/paths", // 开发环境相对路径
      "deploy/autos/paths" // 另一种可能的路径
    };

    for (String basePath : possiblePaths) {
      java.io.File pathsDir = new java.io.File(basePath);
      if (pathsDir.exists() && pathsDir.isDirectory()) {
        java.io.File[] jsonFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".json"));
        if (jsonFiles != null && jsonFiles.length > 0) {
          for (java.io.File file : jsonFiles) {
            String pathName = file.getName().replace(".json", "");
            blinePathChooser.addOption(pathName, pathName);
            foundPaths = true;
          }
          break;
        }
      }
    }

    // 方法2: 如果文件系统方法失败，尝试从classpath资源读取
    if (!foundPaths) {
      try {
        java.net.URL resourceUrl = getClass().getClassLoader().getResource("autos/paths");
        if (resourceUrl != null) {
          java.io.File pathsDir = new java.io.File(resourceUrl.getPath());
          if (pathsDir.exists() && pathsDir.isDirectory()) {
            java.io.File[] jsonFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".json"));
            if (jsonFiles != null) {
              for (java.io.File file : jsonFiles) {
                String pathName = file.getName().replace(".json", "");
                blinePathChooser.addOption(pathName, pathName);
                foundPaths = true;
              }
            }
          }
        }
      } catch (Exception e) {
        // 忽略错误，继续使用默认选项
      }
    }

    // 如果仍然没有找到路径，手动添加已知路径（基于项目结构）
    if (!foundPaths) {
      // 这些是项目中的已知路径文件
      String[] knownPaths = {
        "auto_Down", "example_a", "example_b", "multi_point", "ranged_constraints", "simple_move"
      };
      for (String pathName : knownPaths) {
        blinePathChooser.addOption(pathName, pathName);
      }
    }

    // 注册到NetworkTable使其显示在dashboard上
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
        "BLine Path Chooser", blinePathChooser);
  }

  /** 自动系统选择器 - 选择使用哪个系统 */
  private final SendableChooser<String> systemChooser = new SendableChooser<>();

  /** 配置系统选择器 */
  private void configureSystemChooser() {
    // 添加系统选项
    systemChooser.addOption("PathPlanner", "pathplanner");
    systemChooser.addOption("BLine", "bline");
    systemChooser.addOption("None (Stop)", "none");

    // 设置默认
    systemChooser.setDefaultOption("PathPlanner", "pathplanner");

    // 注册到NetworkTable
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
        "Auto System Chooser", systemChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Y button: Follow example path
    // 注意：BLine路径跟随命令应该在自动模式或需要时调用getBLineCommand()方法
    // 这里暂时禁用，因为路径文件需要在运行时加载

    // SysId tests for shooter subsystems
    // controller.y().whileTrue(shooter.getHood().sysId());
  }

  /**
   * Get the BLinePathFollower subsystem
   *
   * @return BLinePathFollower instance
   */
  public BLinePathFollower getBLinePathFollower() {
    return blinePathFollower;
  }

  /**
   * Get the Drive subsystem
   *
   * @return Drive instance
   */
  public Drive getDrive() {
    return drive;
  }

  // /**
  //  * Get the Feeder subsystem
  //  *
  //  * @return FeederSubsystem instance
  //  */
  // public FeederSubsystem getFeeder() {
  //   return feeder;
  // }

  // /**
  //  * Get the Intake subsystem
  //  *
  //  * @return IntakeSubsystem instance
  //  */
  // public IntakeSubsystem getIntake() {
  //   return intake;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 获取选择的系统
    String selectedSystem = systemChooser.getSelected();

    // 默认为PathPlanner
    if (selectedSystem == null) {
      selectedSystem = "pathplanner";
    }

    // 根据选择的系统执行
    switch (selectedSystem) {
      case "pathplanner":
        // PathPlanner系统
        Command ppCommand = autoChooser.get();
        if (ppCommand != null) {
          return ppCommand;
        }
        // 如果PathPlanner返回空命令，返回停止
        return Commands.runOnce(drive::stop, drive);

      case "bline":
        // BLine系统
        String blinePath = blinePathChooser.getSelected();
        if (blinePath != null && !blinePath.isEmpty()) {
          return getBLineCommand(blinePath);
        }
        // 如果BLine没有选择，返回停止
        return Commands.runOnce(drive::stop, drive);

      case "none":
      default:
        // None - 停止
        return Commands.runOnce(drive::stop, drive);
    }
  }

  /**
   * 构建BLine路径跟随命令（供自动模式使用）
   *
   * @param pathFilename 路径文件名（不含.json扩展名）
   * @return 路径跟随命令，如果路径为空则返回停止命令
   */
  public Command getBLineCommand(String pathFilename) {
    // 验证路径文件名
    if (pathFilename == null || pathFilename.isEmpty()) {
      return Commands.runOnce(drive::stop, drive);
    }

    try {
      // 加载路径
      Path path = blinePathFollower.loadPath(pathFilename);

      // 验证路径是否成功加载
      if (path == null) {
        System.err.println("BLine: Failed to load path - " + pathFilename);
        return Commands.runOnce(drive::stop, drive);
      }

      // Pre-Match Module Orientation: 获取初始模块方向并在命令开始前设置
      // 这可以防止自动开始时模块旋转导致的微小偏差
      Rotation2d initialDirection = path.getInitialModuleDirection();

      // 构建命令（带姿态重置和模块方向预设置）
      Command followCommand = blinePathFollower.buildFollowCommandWithPoseReset(path);

      // 在命令开始时预设置模块方向
      return Commands.sequence(
          Commands.runOnce(() -> drive.setModuleOrientations(initialDirection)), followCommand);
    } catch (Exception e) {
      System.err.println(
          "BLine: Error building command for " + pathFilename + ": " + e.getMessage());
      return Commands.runOnce(drive::stop, drive);
    }
  }
}
