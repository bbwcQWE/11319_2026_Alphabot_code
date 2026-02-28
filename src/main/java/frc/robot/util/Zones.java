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
 * A collection of zone-related classes for defining and checking robot zones on the field.
 *
 * <p>Provides both static zone checking and predictive zone checking based on robot velocity.
 */
public class Zones {

  /** Interface for defining a zone that can be checked against a pose. */
  public static interface Zone {
    /**
     * Checks if the given pose is within this zone.
     *
     * @param pose Supplier of the pose to check
     * @return Trigger that is active when the pose is within the zone
     */
    Trigger contains(Supplier<Pose2d> pose);

    /**
     * Checks if the given translation is within this zone.
     *
     * @param translation Supplier of the translation to check
     * @return Trigger that is active when the translation is within the zone
     */
    Trigger containsTranslation(Supplier<Translation2d> translation);
  }

  /**
   * Interface for defining a predictive zone that can check if a pose will be within the zone after
   * a given time delta based on field-relative speeds.
   */
  public static interface PredictiveXZone extends Zone {
    /**
     * Checks if the pose will be within this zone after the given time delta, accounting for the
     * robot's field-relative velocity.
     *
     * @param pose Supplier of the current pose
     * @param fieldSpeeds Supplier of the field-relative chassis speeds
     * @param dt Time delta to predict ahead
     * @return Trigger that will be active when the predicted pose is within the zone
     */
    Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);

    /**
     * Checks if the translation will be within this zone after the given time delta, accounting for
     * the field-relative velocity.
     *
     * @param translation Supplier of the current translation
     * @param fieldSpeeds Supplier of the field-relative chassis speeds
     * @param dt Time delta to predict ahead
     * @return Trigger that will be active when the predicted translation is within the zone
     */
    Trigger willContainTranslation(
        Supplier<Translation2d> translation, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
  }

  /** Base implementation of a rectangular zone on the field. */
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

  /**
   * Predictive implementation of a rectangular zone that can predict future positions based on
   * field-relative chassis speeds.
   */
  public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {

    /**
     * Creates a predictive zone with the given bounds.
     *
     * @param xMin Minimum X bound in meters
     * @param xMax Maximum X bound in meters
     * @param yMin Minimum Y bound in meters
     * @param yMax Maximum Y bound in meters
     */
    public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    /**
     * Creates a predictive zone with the given bounds using Distance units.
     *
     * @param xMin Minimum X bound
     * @param xMax Maximum X bound
     * @param yMin Minimum Y bound
     * @param yMax Maximum Y bound
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

  /**
   * A collection of zones that treats them as a single zone using OR logic. If any zone contains
   * the pose, the collection contains the pose.
   */
  public static class ZoneCollection implements Zone {
    private final List<Zone> zones = new ArrayList<>();

    /**
     * Adds a zone to this collection.
     *
     * @param zone Zone to add
     * @return This collection for chaining
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

  /**
   * A collection of predictive zones that treats them as a single predictive zone using OR logic.
   * If any zone will contain the predicted pose, the collection will contain the pose.
   */
  public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {

    /**
     * Adds a predictive zone to this collection.
     *
     * @param zone Predictive zone to add
     * @return This collection for chaining
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
