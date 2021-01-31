package frc.robot.subsystems;

public class SubsystemsInstance {
    public DriveSubsystem m_driveSubsystem;
    public TurretSubsystem m_turretSubsystem;

    private static SubsystemsInstance inst;

    private SubsystemsInstance() {
        m_driveSubsystem = new DriveSubsystem();
        m_turretSubsystem = new TurretSubsystem();
    }

    public static SubsystemsInstance getInstance() {
        if(inst == null) inst = new SubsystemsInstance();

        return inst;
    }
}