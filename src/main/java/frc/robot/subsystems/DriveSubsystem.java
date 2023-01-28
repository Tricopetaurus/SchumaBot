// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODOs and ASSUMPTIONS
 * - TODO: Check if we're using BRUSHLESS or BRUSHED motors
 * - TODO: Implement PID
 *    - https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
 * - TODO: Add odometer, making use of our encoder and gyro
 */
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.Drive;
import frc.robot.utils.PIDHelper;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_left_motor_primary = new CANSparkMax(Drive.LEFT_PRIMARY_PORT, MotorType.kBrushless);
  private final CANSparkMax m_left_motor_secondary = new CANSparkMax(Drive.LEFT_SECONDARY_PORT, MotorType.kBrushless);
  private final CANSparkMax m_right_motor_primary = new CANSparkMax(Drive.RIGHT_PRIMARY_PORT, MotorType.kBrushless);
  private final CANSparkMax m_right_motor_secondary = new CANSparkMax(Drive.RIGHT_SECONDARY_PORT, MotorType.kBrushless);
  private final CANSparkMax[] all_motors = {
    m_left_motor_primary,
    m_right_motor_primary,
    m_left_motor_secondary,
    m_right_motor_secondary
  };
  private double m_max_output = Drive.DEFAULT_MAX_OUTPUT;

  private final RelativeEncoder m_left_encoder;
  private final RelativeEncoder m_right_encoder;
  private final AHRS m_nav = new AHRS(SPI.Port.kMXP);

  private final PIDHelper m_left_pid;
  private final PIDHelper m_right_pid;
  private final boolean m_use_pid;
  private final boolean m_tune_pid;

  public DriveSubsystem() {
    this(false, false);
  }

  public DriveSubsystem(boolean use_pid, boolean tune_pid) {

    // Reset all motors to their defaults
    for (CANSparkMax m: all_motors) {
      m.restoreFactoryDefaults();
    }

    m_use_pid = use_pid;
    m_tune_pid = tune_pid;

    // Set up follow so that we only need to control one motor per side
    m_left_motor_secondary.follow(m_left_motor_primary);
    m_right_motor_secondary.follow(m_right_motor_primary);

    m_left_encoder = m_left_motor_primary.getEncoder();
    m_right_encoder = m_right_motor_primary.getEncoder();

    if (m_use_pid) {
      m_left_pid = new PIDHelper("Left ", m_left_motor_primary.getPIDController());
      m_right_pid = new PIDHelper("Right", m_right_motor_primary.getPIDController());
      if (m_tune_pid) {
        m_left_pid.show_on_smartdash();
        m_right_pid.show_on_smartdash();
      }
    }
    else {
      m_left_pid = null;
      m_right_pid = null;
    }
  }

  public void drive(double left, double right) {
    // Inputs are assumed to scale between -1.0, 1.0
    // First, apply deadband
    left = MathUtil.applyDeadband(left, Drive.DEFAULT_DEADBAND);
    right = MathUtil.applyDeadband(right, Drive.DEFAULT_DEADBAND);
    // Clamp to a standard range of -1.0, 1.0
    left = MathUtil.clamp(left, -1.0, 1.0);
    right = MathUtil.clamp(right, -1.0, 1.0);
    // Then square the inputs to get a nicer curve
    left = Math.copySign(left*left, left);
    right = Math.copySign(right*right, right);
    // Finally, scale by our set max value
    left = left * m_max_output;
    right = right * m_max_output;
    if (m_use_pid) {
      m_left_pid.set(left);
      m_right_pid.set(right);
    }
    else {
      m_left_motor_primary.set(left);
      m_right_motor_primary.set(right);
    }
  }

  public void set_maximum(double maximum) {
    if (maximum < 0 || maximum > 1) {
      System.err.println("Maximum call out of bounds!");
    }
    m_max_output = maximum;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left RPM", m_left_encoder.getVelocity());
    SmartDashboard.putNumber("Right RPM", m_right_encoder.getVelocity());
    if (m_tune_pid) {
      m_left_pid.update_from_smartdash();
      m_right_pid.update_from_smartdash();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
