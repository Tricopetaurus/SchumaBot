package frc.robot.utils;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDHelper {
    private double p, i, d, iz, ff;
    private double max_output, min_output, max_vel, min_vel, max_acc;
    private double err_tolerance;
    private final String m_name;
    private final SparkMaxPIDController m_pid_controller;

    public PIDHelper(String name, SparkMaxPIDController pid_controller) {
      /* TODO: Kick these down to constants */
      /* Defaults pulled from example code https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java */
      p = 5e-6;
      i = 1e-6;
      d = 0;
      iz = 0;
      ff = 0.000156;
      max_output = 1;
      min_output = -1;
      max_vel = 2000;
      max_acc = 1500;

      err_tolerance = 0;
      min_vel = 0;

      m_name = name;
      m_pid_controller = pid_controller;

      flush_parameters();
    }

    // Should only have to call this once when in init
    public void show_on_smartdash() {
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber(m_name+"P Gain", p);
      SmartDashboard.putNumber(m_name+"I Gain", i);
      SmartDashboard.putNumber(m_name+"D Gain", d);
      SmartDashboard.putNumber(m_name+"I Zone", iz);
      SmartDashboard.putNumber(m_name+"Feed Forward", ff);
      SmartDashboard.putNumber(m_name+"Max Output", max_output);
      SmartDashboard.putNumber(m_name+"Min Output", min_output);

      // display Smart Motion coefficients
      SmartDashboard.putNumber(m_name + "Max Velocity", max_vel);
      SmartDashboard.putNumber(m_name + "Min Velocity", min_vel);
      SmartDashboard.putNumber(m_name + "Max Acceleration", max_acc);
      SmartDashboard.putNumber(m_name + "Allowed Closed Loop Error", err_tolerance);
      SmartDashboard.putNumber(m_name + "Set Velocity", 0);
    }

    public void flush_parameters() {
      m_pid_controller.setP(p);
      m_pid_controller.setI(i);
      m_pid_controller.setD(d);
      m_pid_controller.setIZone(iz);
      m_pid_controller.setFF(ff);
      m_pid_controller.setOutputRange(min_output, max_output);
      m_pid_controller.setSmartMotionMaxVelocity(max_vel, 0);
      m_pid_controller.setSmartMotionMinOutputVelocity(min_vel, 0);;
      m_pid_controller.setSmartMotionMaxAccel(max_acc, 0);
      m_pid_controller.setSmartMotionAllowedClosedLoopError(err_tolerance, 0);
    }

    public void set(double set_point) {
      // m_pid_controller.setReference(set_point, CANSparkMax.ControlType.kSmartVelocity); <-- ? Is this better ?
      this.m_pid_controller.setReference(set_point, CANSparkMax.ControlType.kDutyCycle);
    }

    public void update_from_smartdash() {
      boolean modified = false;
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber(m_name + "P Gain", 0);
      double i = SmartDashboard.getNumber(m_name + "I Gain", 0);
      double d = SmartDashboard.getNumber(m_name + "D Gain", 0);
      double iz = SmartDashboard.getNumber(m_name + "I Zone", 0);
      double ff = SmartDashboard.getNumber(m_name + "Feed Forward", 0);
      double max = SmartDashboard.getNumber(m_name + "Max Output", 0);
      double min = SmartDashboard.getNumber(m_name + "Min Output", 0);
      double maxV = SmartDashboard.getNumber(m_name + "Max Velocity", 0);
      double minV = SmartDashboard.getNumber(m_name + "Min Velocity", 0);
      double maxA = SmartDashboard.getNumber(m_name + "Max Acceleration", 0);
      double allE = SmartDashboard.getNumber(m_name + "Allowed Closed Loop Error", 0);

      // TODO: This can definitely be improved with a mapping or array...
      if (this.p != p) { this.p = p; modified = true; }
      if (this.i != i) { this.i = i; modified = true; }
      if (this.d != d) { this.d = d; modified = true; }
      if (this.iz != iz) { this.iz = iz; modified = true; }
      if (this.ff != ff) { this.ff = ff; modified = true; }
      if (this.max_output != max) { this.max_output = max; modified = true; }
      if (this.min_output != min) { this.min_output = min; modified = true; }
      if (this.max_vel != maxV) { this.max_vel = maxV; modified = true; }
      if (this.min_vel != minV) { this.min_vel = minV; modified = true; }
      if (this.max_acc != maxA) { this.max_acc = maxA; modified = true; }
      if (this.err_tolerance != allE) { this.err_tolerance = allE; modified = true; }

      if (modified) {
        flush_parameters();
      }
    }
  }