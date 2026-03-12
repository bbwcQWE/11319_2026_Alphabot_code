// Copyright (c) 2025-2026 11319 Polaris
// https://github.com/bbwcQWE
//
// QuestNav 硬件 IO 实现 - 从 Quest 头显读取姿态数据并发送重置命令

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;

/** QuestNav 硬件的 IO 实现 */
public class VisionIOQuestNav implements VisionIO {
  private final QuestNav questNav;
  private final Transform3d robotToQuest;

  /**
   * 创建新的 VisionIOQuestNav
   *
   * @param robotToQuest 从机器人中心到 Quest 头显的变换
   */
  public VisionIOQuestNav(Transform3d robotToQuest) {
    this.robotToQuest = robotToQuest;
    this.questNav = new QuestNav();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // 调用 commandPeriodic 保持 QuestNav 更新
    // QuestNav文档要求：此方法必须在每次周期调用，否则系统无法正常工作
    questNav.commandPeriodic();

    // 获取所有未读的姿态帧
    PoseFrame[] rawFrames = questNav.getAllUnreadPoseFrames();

    // 更新连接状态 - 必须检查是否正在追踪
    // 根据QuestNav文档：最后发布的值会持久化，即使Quest断开连接
    // 因此必须确保Quest连接且正在追踪才认为可用
    boolean isTracking = rawFrames.length > 0 && rawFrames[rawFrames.length - 1].isTracking();
    inputs.connected = isTracking;

    // 转换为 PoseObservation 格式
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (PoseFrame frame : rawFrames) {
      // 只处理正在追踪的帧
      if (frame.isTracking()) {
        // 获取 Quest 姿态
        Pose3d questPose = frame.questPose3d();

        // 转换为机器人姿态
        Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

        poseObservations.add(
            new PoseObservation(
                frame.dataTimestamp(), // 时间戳
                robotPose, // 3D 姿态
                0.0, // 歧义度 - QuestNav 不使用歧义度
                1, // 标签数量 - QuestNav 使用内部追踪
                0.0, // 平均标签距离 - 不适用
                PoseObservationType.MEGATAG_2 // 视为 MegaTag2 类型
                ));
      }
    }

    // 保存姿态观测
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // 无标签 ID（QuestNav 使用内部追踪）
    inputs.tagIds = new int[0];
  }

  /**
   * 重置 QuestNav 姿态到指定的机器人位置
   *
   * @param robotPose 机器人在场地上的已知位置
   */
  public void resetPose(Pose3d robotPose) {
    // 将机器人姿态转换为 Quest 姿态
    // 注意：这里不使用 inverse，因为我们要从机器人中心推到 Quest 位置
    Pose3d questPose = robotPose.transformBy(robotToQuest);

    // 发送重置命令到 QuestNav
    questNav.setPose(questPose);

    // 调用 commandPeriodic 确保重置命令被处理
    questNav.commandPeriodic();
  }

  /**
   * 检查 QuestNav 是否正在追踪
   *
   * @return 如果正在追踪返回 true
   */
  public boolean isTracking() {
    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    return frames.length > 0 && frames[frames.length - 1].isTracking();
  }

  /**
   * 获取原始 QuestNav 实例（用于高级操作）
   *
   * @return QuestNav 实例
   */
  public QuestNav getQuestNav() {
    return questNav;
  }
}
