// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
// 发射器子系统 - Turret（炮塔）

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {

  @AutoLog
  public static class TurretInputs {
    public Angle position = Degrees.of(0);
    public double voltage = 0;
    public double current = 0;
  }

  private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();

  private final TalonFX turretMotor = new TalonFX(19);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private final SmartMotorController turretSMC =
      new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final PivotConfig turretConfig =
      new PivotConfig(turretSMC)
          .withStartingPosition(Degrees.of(0))
          .withHardLimit(Degrees.of(0), Degrees.of(720))
          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH)
          .withMOI(Meters.of(0.25), Pounds.of(4));

  private final Pivot turret = new Pivot(turretConfig);

  private void updateInputs() {
    turretInputs.position = turret.getAngle();
    turretInputs.voltage = turretSMC.getVoltage().in(Volts);
    turretInputs.current = turretSMC.getStatorCurrent().in(Amps);
  }

  public TurretSubsystem() {}

  public Command setAngle(Angle angle) {
    Logger.recordOutput("Turret/Setpoint", angle);
    return turret.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    Logger.recordOutput("Turret/Setpoint", angle);
    turretSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(
        () -> {
          Angle angle = angleSupplier.get();
          Logger.recordOutput("Turret/Setpoint", angle);
          return angle;
        });
  }

  public Angle getAngle() {
    return turretInputs.position;
  }

  public Command sysId() {
    return turret.sysId(Volts.of(4.0), Volts.per(Second).of(0.5), Seconds.of(8.0));
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Turret", turretInputs);
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
