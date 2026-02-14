// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  @AutoLog
  public static class ShooterStateInputs {
    public boolean isReady = false;
    public boolean isAiming = false;
    public boolean isShooting = false;
    public double targetDistance = 0;
    public double hoodAngle = 0;
    public double turretAngle = 0;
    public double flywheelSpeed = 0;
  }

  private final ShooterStateInputsAutoLogged shooterStateInputs =
      new ShooterStateInputsAutoLogged();

  private final HoodSubsystem hood = new HoodSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final FlyWheelSubsystem flywheel = new FlyWheelSubsystem();

  private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> DegreesPerSecond.of(0);

  private boolean isReady = false;
  private boolean isAiming = false;
  private boolean isShooting = false;
  private double targetDistance = 0;

  private static final double HOOD_MIN_ANGLE = 20.0;
  private static final double HOOD_MAX_ANGLE = 70.0;
  private static final double FLYWHEEL_MIN_RPM = 1000.0;
  private static final double FLYWHEEL_MAX_RPM = 6000.0;

  private void updateInputs() {
    shooterStateInputs.isReady = isReady;
    shooterStateInputs.isAiming = isAiming;
    shooterStateInputs.isShooting = isShooting;
    shooterStateInputs.targetDistance = targetDistance;
    shooterStateInputs.hoodAngle = hood.getAngle().in(Degrees);
    shooterStateInputs.turretAngle = turret.getAngle().in(Degrees);
    shooterStateInputs.flywheelSpeed = flywheel.getVelocity().in(RPM);
  }

  public ShooterSubsystem() {}

  public Command aimAtTarget(double distance, double targetHeight) {
    isAiming = true;
    Logger.recordOutput("Shooter/TargetDistance", distance);
    Logger.recordOutput("Shooter/TargetHeight", targetHeight);

    double[] shotParams = calculateShotParameters(distance, targetHeight);
    double hoodAngleDeg = shotParams[0];
    double flywheelRPM = shotParams[1];

    Angle hoodAngle = Degrees.of(hoodAngleDeg);
    AngularVelocity flywheelSpeed = RPM.of(flywheelRPM);

    flywheelVelocitySupplier = () -> flywheelSpeed;

    return hood.setAngle(hoodAngle)
        .alongWith(turret.setAngle(Degrees.of(0)))
        .beforeStarting(() -> isAiming = true, hood, turret)
        .andThen(
            () -> {
              isAiming = false;
              isReady = true;
            });
  }

  private double[] calculateShotParameters(double distance, double targetHeight) {
    double hoodAngle;
    double flywheelRPM;

    if (distance < 2.0) {
      hoodAngle = 65.0;
      flywheelRPM = 2000.0;
    } else if (distance < 3.0) {
      hoodAngle = 55.0;
      flywheelRPM = 3000.0;
    } else if (distance < 4.0) {
      hoodAngle = 45.0;
      flywheelRPM = 4000.0;
    } else if (distance < 5.0) {
      hoodAngle = 38.0;
      flywheelRPM = 4800.0;
    } else if (distance < 6.0) {
      hoodAngle = 32.0;
      flywheelRPM = 5200.0;
    } else if (distance < 7.0) {
      hoodAngle = 28.0;
      flywheelRPM = 5500.0;
    } else if (distance < 8.0) {
      hoodAngle = 25.0;
      flywheelRPM = 5700.0;
    } else {
      hoodAngle = 22.0;
      flywheelRPM = 5800.0;
    }

    hoodAngle = Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, hoodAngle));
    flywheelRPM = Math.max(FLYWHEEL_MIN_RPM, Math.min(FLYWHEEL_MAX_RPM, flywheelRPM));

    return new double[] {hoodAngle, flywheelRPM};
  }

  public Command prepareToShoot() {
    return flywheel
        .setVelocity(flywheelVelocitySupplier.get())
        .beforeStarting(
            () -> {
              isReady = false;
              isShooting = true;
            },
            flywheel);
  }

  public boolean isReadyToShoot() {
    double currentRPM = flywheel.getVelocity().in(RPM);
    double targetRPM = flywheelVelocitySupplier.get().in(RPM);
    double rpmError = Math.abs(currentRPM - targetRPM);

    double hoodError =
        Math.abs(hood.getAngle().in(Degrees) - calculateShotParameters(targetDistance, 0)[0]);

    return rpmError < 200 && hoodError < 5 && !isShooting;
  }

  public Command executeShoot() {
    return prepareToShoot()
        .andThen(
            () -> {
              isShooting = false;
              isReady = false;
            });
  }

  public Command aimAt(Angle hoodAngle, Angle turretAngle) {
    return hood.setAngle(hoodAngle).alongWith(turret.setAngle(turretAngle));
  }

  public Command runShooter() {
    if (flywheelVelocitySupplier == null) {
      DriverStation.reportWarning("Shooter velocity set to null, not running shooter", true);
      return flywheel.setDutyCycle(0);
    }
    return flywheel.setVelocity(flywheelVelocitySupplier.get());
  }

  public Command stopShooter() {
    return flywheel.setVelocity(DegreesPerSecond.of(0));
  }

  public Command runShooter(AngularVelocity velocity) {
    if (velocity == null) {
      DriverStation.reportWarning("Shooter velocity set to null, defaulting to 0", true);
      velocity = DegreesPerSecond.of(0);
    }
    return flywheel.setVelocity(velocity);
  }

  public void setVelocitySupplier(Supplier<AngularVelocity> velocitySupplier) {
    flywheelVelocitySupplier = velocitySupplier;
  }

  public HoodSubsystem getHood() {
    return hood;
  }

  public TurretSubsystem getTurret() {
    return turret;
  }

  public FlyWheelSubsystem getFlywheel() {
    return flywheel;
  }

  public void setReady(boolean ready) {
    isReady = ready;
  }

  public void setAiming(boolean aiming) {
    isAiming = aiming;
  }

  public void setShooting(boolean shooting) {
    isShooting = shooting;
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Shooter/State", shooterStateInputs);
  }
}
