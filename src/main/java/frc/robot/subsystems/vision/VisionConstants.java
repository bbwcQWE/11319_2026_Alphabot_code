/**
 * 视觉常量 - 配置视觉子系统的参数
 */
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * 视觉常量 - 配置视觉子系统的参数
 */
public class VisionConstants {
  // AprilTag场地布局
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // 相机名称，必须与处理器上配置的名称匹配
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // 机器人到相机的变换
  // (Limelight不使用，请在Web UI中配置)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // 基本过滤阈值
  public static double maxAmbiguity = 0.3;    // 最大歧义度
  public static double maxZError = 0.75;       // 最大Z轴误差

  // 标准差基线（1米距离和1个标签）
  // （根据距离和标签数量自动调整）
  public static double linearStdDevBaseline = 0.02; // 米
  public static double angularStdDevBaseline = 0.06; // 弧度

  // 每个相机的标准差乘数
  // （调整以信任某些相机多于其他相机）
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // 相机 0
        1.0  // 相机 1
      };

  // MegaTag 2观测的乘数
  public static double linearStdDevMegatag2Factor = 0.5; // 比完整3D解算更稳定
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // 无旋转数据
}
