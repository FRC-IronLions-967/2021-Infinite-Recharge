package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SubsystemsInstance {
    public DriveSubsystem m_driveSubsystem;
    public TurretSubsystem m_turretSubsystem;
    public ShooterSubsystem m_shooterSubsystem;

    private static SubsystemsInstance inst;

    private SubsystemsInstance() {
        m_driveSubsystem = new DriveSubsystem();
        m_turretSubsystem = new TurretSubsystem();
        m_shooterSubsystem = new ShooterSubsystem();

        CommandScheduler.getInstance().registerSubsystem(m_driveSubsystem);
        CommandScheduler.getInstance().registerSubsystem(m_turretSubsystem);
        CommandScheduler.getInstance().registerSubsystem(m_shooterSubsystem);
    }

    public static SubsystemsInstance getInstance() {
        if(inst == null) inst = new SubsystemsInstance();

        return inst;
    }
}