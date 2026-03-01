// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * 视觉IO接口 - 定义视觉子系统的输入输出
 */
public interface VisionIO {
  /** 视觉输入数据类 */
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;                           // 相机连接状态
    public TargetObservation latestTargetObservation =          // 最新的目标观测
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public PoseObservation[] poseObservations = new PoseObservation[0]; // 姿态观测数组
    public int[] tagIds = new int[0];                           // 检测到的标签ID
  }

  /** 表示简单目标的角度，不用于姿态估计 */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** 表示用于姿态估计的机器人姿态样本 */
  public static record PoseObservation(
      double timestamp,            // 时间戳
      Pose3d pose,                // 姿态
      double ambiguity,           // 歧义度
      int tagCount,               // 标签数量
      double averageTagDistance,  // 平均标签距离
      PoseObservationType type) {} // 观测类型

  /** 姿态观测类型枚举 */
  public static enum PoseObservationType {
    MEGATAG_1,       // MegaTag 1算法
    MEGATAG_2,       // MegaTag 2算法
    PHOTONVISION     // PhotonVision估计
  }

  /** 更新输入数据直接 */
  public default void updateInputs(VisionIOInputs inputs) {}
}
