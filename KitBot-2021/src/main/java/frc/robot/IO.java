package frc.robot;

import frc.robot.utils.controls.XBoxController;

public class IO {
    private static IO instance;

    private XBoxController driverController;
    private XBoxController manipulatorController;

    private IO() {
        driverController = new XBoxController(0);
        manipulatorController = new XBoxController(1);
    }

    public static IO getInstance() {
        if(instance == null) instance = new IO();

        return instance;
    }

    public void telopInit() {

    }

    public XBoxController getDriverController() {
        return driverController;
    }

    public XBoxController getManipulatorController() {
        return manipulatorController;
    }
}