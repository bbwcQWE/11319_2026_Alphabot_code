// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Shooter extends SubsystemBase {

  @AutoLog
  public static class ShooterInputs {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();

  private SmartMotorControllerConfig shooterSmcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withFollowers(Pair.of(new TalonFX(17), true));

  private TalonFX shooterMotorLeft = new TalonFX(16);

  private SmartMotorController shooterMotor =
      new TalonFXWrapper(shooterMotorLeft, DCMotor.getKrakenX60(1), shooterSmcConfig);
  private final FlyWheelConfig shooterConfig =
      new FlyWheelConfig(shooterMotor)
          .withDiameter(Inches.of(4))
          .withMass(Kilograms.of(2))
          .withUpperSoftLimit(RPM.of(5000));

  private FlyWheel shooterFlyWheel = new FlyWheel(shooterConfig);

  private void updateInputs() {
    shooterInputs.velocity = shooterFlyWheel.getSpeed();
    shooterInputs.setpoint = shooterMotor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    shooterInputs.volts = shooterMotor.getVoltage();
    shooterInputs.current = shooterMotor.getStatorCurrent();
  }

  public AngularVelocity getVelocity() {
    return shooterInputs.velocity;
  }

  public Command setVelocity(AngularVelocity speed) {
    Logger.recordOutput("Shooter/Setpoint", speed);
    return shooterFlyWheel.run(speed);
  }

  public void setVelocitySetpoint(AngularVelocity speed) {
    Logger.recordOutput("Shooter/Setpoint", speed);
    shooterFlyWheel.setMechanismVelocitySetpoint(speed);
  }

  public Command set(double dutyCycle) {
    Logger.recordOutput("Shooter/DutyCycle", dutyCycle);
    return shooterFlyWheel.set(dutyCycle);
  }

  public Shooter() {}

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Shooter", shooterInputs);
    shooterFlyWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooterFlyWheel.simIterate();
  }
}
