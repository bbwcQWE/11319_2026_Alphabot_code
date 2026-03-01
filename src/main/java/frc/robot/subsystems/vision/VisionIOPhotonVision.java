// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** PhotonVision硬件的IO实现 */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * 创建新的VisionIOPhotonVision
   *
   * @param name 相机的配置名称
   * @param robotToCamera 相机相对于机器人的3D位置
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // 读取新的相机观测数据
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // 更新最新目标观测
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      // 添加姿态观测
      if (result.multitagResult.isPresent()) { // 多标签结果
        var multitagResult = result.multitagResult.get();

        // 计算机器人姿态
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // 计算平均标签距离
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // 添加标签ID
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // 添加观测
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // 时间戳
                robotPose, // 3D姿态估计
                multitagResult.estimatedPose.ambiguity, // 歧义度
                multitagResult.fiducialIDsUsed.size(), // 标签数量
                totalTagDistance / result.targets.size(), // 平均标签距离
                PoseObservationType.PHOTONVISION)); // 观测类型

      } else if (!result.targets.isEmpty()) { // 单标签结果
        var target = result.targets.get(0);

        // 计算机器人姿态
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // 添加标签ID
          tagIds.add((short) target.fiducialId);

          // 添加观测
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // 时间戳
                  robotPose, // 3D姿态估计
                  target.poseAmbiguity, // 歧义度
                  1, // 标签数量
                  cameraToTarget.getTranslation().getNorm(), // 平均标签距离
                  PoseObservationType.PHOTONVISION)); // 观测类型
        }
      }
    }

    // 保存姿态观测到输入对象
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // 保存标签ID到输入对象
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
