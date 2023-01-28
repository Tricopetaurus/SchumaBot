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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.Drive;

import com.kauailabs.navx.frc.AHRS;

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

  private final RelativeEncoder m_left_encoder;
  private final RelativeEncoder m_right_encoder;
  private final AHRS m_nav = new AHRS(SPI.Port.kMXP);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_left_motor_primary, m_right_motor_primary);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    // Reset all motors to their defaults
    for (CANSparkMax m: all_motors) {
      m.restoreFactoryDefaults();
    }

    // Set up follow so that we only need to control one motor per side
    m_left_motor_secondary.follow(m_left_motor_primary);
    m_right_motor_secondary.follow(m_right_motor_primary);

    m_left_encoder = m_left_motor_primary.getEncoder();
    m_right_encoder = m_right_motor_primary.getEncoder();
  }


  public void drive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void set_maximum(double maximum) {
    if (maximum < 0 || maximum > 1) {
      System.err.println("Maximum call out of bounds!");
    }
    m_drive.setMaxOutput(maximum);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left RPM", m_left_encoder.getVelocity());
    SmartDashboard.putNumber("Right RPM", m_right_encoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
