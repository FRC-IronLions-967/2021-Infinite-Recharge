package frc.robot.subsystems;

public class SubsystemsInstance {
    public DriveSubsystem m_driveSubsystem;

    private static SubsystemsInstance inst;

    private SubsystemsInstance() {
        m_driveSubsystem = new DriveSubsystem();
    }

    public static SubsystemsInstance getInstance() {
        if(inst == null) inst = new SubsystemsInstance();

        return inst;
    }
}