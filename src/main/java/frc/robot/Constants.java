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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class FieldConstants {
    // FRC场地尺寸 (英寸)
    public static final double FIELD_LENGTH_INCHES = 650.12;
    public static final double FIELD_WIDTH_INCHES = 316.64;

    // 转换为米
    public static final Distance FIELD_LENGTH = Inches.of(FIELD_LENGTH_INCHES);
    public static final Distance FIELD_WIDTH = Inches.of(FIELD_WIDTH_INCHES);

    // trench区域 (需要根据你的机器人实际场地调整)
    public static final double TRENCH_X_START = 0.0; // 米
    public static final double TRENCH_X_END = 5.5;
    public static final double TRENCH_Y_MIN = 2.5;
    public static final double TRENCH_Y_MAX = 4.5;

    // 预测时间
    public static final Time DUCK_TIME = Seconds.of(0.2);

    // Hood角度
    public static final Angle MIN_HOOD_ANGLE = Degrees.of(14); // 收回位置
    public static final Angle DEFAULT_HOOD_ANGLE = Degrees.of(45); // 默认位置

    // 炮塔相对于机器人中心的偏移量（米）
    // 正X向右，正Y向上（基于机器人视角）
    public static final double TURRET_OFFSET_X = 1.0;
    public static final double TURRET_OFFSET_Y = 1.0;
    public static final Translation2d TURRET_OFFSET =
        new Translation2d(TURRET_OFFSET_X, TURRET_OFFSET_Y);
  }
}
