package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

// This is going to be pedantic, could be implemented
// just as easily with something like the following off in RobotContainer:
//    m_drive_subsystem.setDefaultCommand(
//        Commands.run(
//            () ->
//                m_drive_subsystem.drive(
//                   m_controller.getLeftY(), m_controller.getRightY()),
//            m_drive_subsystem));

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier left, DoubleSupplier right) {
        m_drive = subsystem;
        m_left = left;
        m_right = right;
        addRequirements(m_drive);
    }
    
    @Override
    public void execute() {
        m_drive.drive(m_left.getAsDouble(), m_right.getAsDouble());
    }
}
