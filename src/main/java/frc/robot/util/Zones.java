/**
 * 区域工具类 - 用于定义和检查机器人场地区域
 *
 * <p>提供静态区域检测和基于机器人速度的预测性区域检测
 */
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * 区域工具类 - 用于定义和检查机器人场地区域
 *
 * <p>提供静态区域检测和基于机器人速度的预测性区域检测
 */
public class Zones {

  /** 定义可检查姿态的区域接口 */
  public static interface Zone {
    /**
     * 检查给定姿态是否在此区域内
     *
     * @param pose 要检查的姿态供应器
     * @return 当姿态在区域内时激活的触发器
     */
    Trigger contains(Supplier<Pose2d> pose);

    /**
     * 检查给定位移是否在此区域内
     *
     * @param translation 要检查的位移供应器
     * @return 当位移在区域内时激活的触发器
     */
    Trigger containsTranslation(Supplier<Translation2d> translation);
  }

  /** 定义预测性区域接口，可根据场地相对速度检查姿态在给定时间后是否会在区域内 */
  public static interface PredictiveXZone extends Zone {
    /**
     * 检查姿态在考虑机器人场地相对速度的情况下，给定时间后是否会在此区域内
     *
     * @param pose 当前姿态的供应器
     * @param fieldSpeeds 场地相对底盘速度的供应器
     * @param dt 预测的时间增量
     * @return 当预测姿态在区域内时将激活的触发器
     */
    Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);

    /**
     * 检查位移在考虑场地相对速度的情况下，给定时间后是否会在此区域内
     *
     * @param translation 当前位移的供应器
     * @param fieldSpeeds 场地相对底盘速度的供应器
     * @param dt 预测的时间增量
     * @return 当预测位移在区域内时将激活的触发器
     */
    Trigger willContainTranslation(
        Supplier<Translation2d> translation, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
  }

  /** 场地矩形区域的基础实现 */
  public static class BaseZone implements Zone {
    protected final double xMin;
    protected final double xMax;
    protected final double yMin;
    protected final double yMax;

    /**
     * Creates a zone with the given bounds.
     *
     * @param xMin Minimum X bound in meters
     * @param xMax Maximum X bound in meters
     * @param yMin Minimum Y bound in meters
     * @param yMax Maximum Y bound in meters
     */
    public BaseZone(double xMin, double xMax, double yMin, double yMax) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
    }

    /**
     * Creates a zone with the given bounds using Distance units.
     *
     * @param xMin Minimum X bound
     * @param xMax Maximum X bound
     * @param yMin Minimum Y bound
     * @param yMax Maximum Y bound
     */
    public BaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      this.xMin = xMin.in(Units.Meters);
      this.xMax = xMax.in(Units.Meters);
      this.yMin = yMin.in(Units.Meters);
      this.yMax = yMax.in(Units.Meters);
    }

    @Override
    public Trigger contains(Supplier<Pose2d> pose) {
      return new Trigger(
          () -> {
            Pose2d currentPose = pose.get();
            if (currentPose == null) {
              return false;
            }
            Translation2d translation = currentPose.getTranslation();
            return translation.getX() >= xMin
                && translation.getX() <= xMax
                && translation.getY() >= yMin
                && translation.getY() <= yMax;
          });
    }

    @Override
    public Trigger containsTranslation(Supplier<Translation2d> translation) {
      return new Trigger(
          () -> {
            Translation2d t = translation.get();
            if (t == null) return false;
            return t.getX() >= xMin && t.getX() <= xMax && t.getY() >= yMin && t.getY() <= yMax;
          });
    }

    /**
     * Creates a mirrored version of this zone across the X axis (center of field).
     *
     * @param fieldWidth Total width of the field in meters
     * @return Mirrored zone
     */
    public Zone mirroredX(double fieldWidth) {
      double centerX = fieldWidth / 2.0;
      double newXMin = centerX - (xMax - centerX);
      double newXMax = centerX - (xMin - centerX);
      return new BaseZone(newXMin, newXMax, yMin, yMax);
    }

    /**
     * Creates a mirrored version of this zone across the Y axis (center of field).
     *
     * @param fieldLength Total length of the field in meters
     * @return Mirrored zone
     */
    public Zone mirroredY(double fieldLength) {
      double centerY = fieldLength / 2.0;
      double newYMin = centerY - (yMax - centerY);
      double newYMax = centerY - (yMin - centerY);
      return new BaseZone(xMin, xMax, newYMin, newYMax);
    }

    /**
     * Gets the four corners of this zone for visualization.
     *
     * @return List of Translation2d representing the corners in order
     */
    public List<Translation2d> getCorners() {
      List<Translation2d> corners = new ArrayList<>();
      corners.add(new Translation2d(xMin, yMin));
      corners.add(new Translation2d(xMax, yMin));
      corners.add(new Translation2d(xMax, yMax));
      corners.add(new Translation2d(xMin, yMax));
      return corners;
    }

    /**
     * Gets the minimum X bound.
     *
     * @return Minimum X in meters
     */
    public double getXMin() {
      return xMin;
    }

    /**
     * Gets the maximum X bound.
     *
     * @return Maximum X in meters
     */
    public double getXMax() {
      return xMax;
    }

    /**
     * Gets the minimum Y bound.
     *
     * @return Minimum Y in meters
     */
    public double getYMin() {
      return yMin;
    }

    /**
     * Gets the maximum Y bound.
     *
     * @return Maximum Y in meters
     */
    public double getYMax() {
      return yMax;
    }
  }

  /** 基于场地相对底盘速度预测未来位置的矩形区域预测实现 */
  public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {

    /**
     * 使用给定边界创建预测区域
     *
     * @param xMin 最小X边界（米）
     * @param xMax 最大X边界（米）
     * @param yMin 最小Y边界（米）
     * @param yMax 最大Y边界（米）
     */
    public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    /**
     * 使用距离单位创建预测区域
     *
     * @param xMin 最小X边界
     * @param xMax 最大X边界
     * @param yMin 最小Y边界
     * @param yMax 最大Y边界
     */
    public PredictiveXBaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      return new Trigger(
          () -> {
            Pose2d currentPose = pose.get();
            ChassisSpeeds speeds = fieldSpeeds.get();

            if (currentPose == null || speeds == null) {
              return false;
            }

            double dtSeconds = dt.in(Units.Seconds);
            Translation2d predictedPosition =
                predictPosition(currentPose.getTranslation(), speeds, dtSeconds);

            return willContainPoint(predictedPosition, speeds, dt);
          });
    }

    @Override
    public Trigger willContainTranslation(
        Supplier<Translation2d> translation, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      return new Trigger(
          () -> {
            Translation2d t = translation.get();
            ChassisSpeeds speeds = fieldSpeeds.get();
            if (t == null || speeds == null) return false;

            double dtSeconds = dt.in(Units.Seconds);
            Translation2d predictedPosition = predictPosition(t, speeds, dtSeconds);

            return willContainPoint(predictedPosition, speeds, dt);
          });
    }

    /**
     * Predicts if a point will be within this zone after the given time delta.
     *
     * @param point The translation to check
     * @param fieldSpeeds The field-relative chassis speeds
     * @param dt Time delta to predict ahead
     * @return True if the predicted position will be within the zone
     */
    protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
      return point.getX() >= xMin
          && point.getX() <= xMax
          && point.getY() >= yMin
          && point.getY() <= yMax;
    }

    /**
     * Predicts the future position based on current position, velocity, and time delta.
     *
     * @param currentPosition Current translation
     * @param fieldSpeeds Field-relative chassis speeds
     * @param dtSeconds Time delta in seconds
     * @return Predicted translation
     */
    private Translation2d predictPosition(
        Translation2d currentPosition, ChassisSpeeds fieldSpeeds, double dtSeconds) {
      double deltaX = fieldSpeeds.vxMetersPerSecond * dtSeconds;
      double deltaY = fieldSpeeds.vyMetersPerSecond * dtSeconds;

      return new Translation2d(currentPosition.getX() + deltaX, currentPosition.getY() + deltaY);
    }
  }

  /** 使用OR逻辑将多个区域视为单个区域的区域集合。如果任何区域包含该姿态，则集合包含该姿态。 */
  public static class ZoneCollection implements Zone {
    private final List<Zone> zones = new ArrayList<>();

    /**
     * 添加区域到此集合
     *
     * @param zone 要添加的区域
     * @return 此集合用于链式调用
     */
    public ZoneCollection add(Zone zone) {
      zones.add(zone);
      return this;
    }

    @Override
    public Trigger contains(Supplier<Pose2d> pose) {
      return new Trigger(
          () -> {
            for (Zone zone : zones) {
              if (zone.contains(pose).getAsBoolean()) {
                return true;
              }
            }
            return false;
          });
    }

    @Override
    public Trigger containsTranslation(Supplier<Translation2d> translation) {
      return new Trigger(
          () -> {
            for (Zone zone : zones) {
              if (zone.containsTranslation(translation).getAsBoolean()) {
                return true;
              }
            }
            return false;
          });
    }

    /**
     * Gets the list of zones in this collection.
     *
     * @return List of zones
     */
    public List<Zone> getZones() {
      return new ArrayList<>(zones);
    }
  }

  /** 使用OR逻辑将多个预测区域视为单个预测区域的预测区域集合。如果任何区域将包含预测的姿态，则集合将包含该姿态。 */
  public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {

    /**
     * 添加预测区域到此集合
     *
     * @param zone 要添加的预测区域
     * @return 此集合用于链式调用
     */
    public PredictiveXZoneCollection add(PredictiveXZone zone) {
      super.add(zone);
      return this;
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      return new Trigger(
          () -> {
            for (Zone zone : getZones()) {
              if (zone instanceof PredictiveXZone) {
                PredictiveXZone predictiveZone = (PredictiveXZone) zone;
                if (predictiveZone.willContain(pose, fieldSpeeds, dt).getAsBoolean()) {
                  return true;
                }
              }
            }
            return false;
          });
    }

    @Override
    public Trigger willContainTranslation(
        Supplier<Translation2d> translation, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      return new Trigger(
          () -> {
            for (Zone zone : getZones()) {
              if (zone instanceof PredictiveXZone) {
                PredictiveXZone predictiveZone = (PredictiveXZone) zone;
                if (predictiveZone
                    .willContainTranslation(translation, fieldSpeeds, dt)
                    .getAsBoolean()) {
                  return true;
                }
              }
            }
            return false;
          });
    }

    /**
     * Gets the list of predictive zones in this collection.
     *
     * @return List of predictive zones
     */
    public List<PredictiveXZone> getPredictiveZones() {
      List<PredictiveXZone> predictiveZones = new ArrayList<>();
      for (Zone zone : getZones()) {
        if (zone instanceof PredictiveXZone) {
          predictiveZones.add((PredictiveXZone) zone);
        }
      }
      return predictiveZones;
    }
  }
}
