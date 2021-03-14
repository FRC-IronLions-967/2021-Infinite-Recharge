package frc.robot.utils.controls;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DoNothingCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;


public class XBoxController extends Joystick {
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int RIGHT_X_AXIS = 4;
    public static final int RIGHT_Y_AXIS = 5;

    //this controls how far you have to press the trigger for it to register as pressed
    private double triggerTolerance = 0.25;

    private HashMap<String, JoystickButton> buttonMap;
    private HashMap<String, POVButton> povMap;

    public XBoxController(int id) {
        super(id);
        buttonMap = new HashMap<>();
        povMap = new HashMap<>();

        //initialize the buttons on the controller
        buttonMap.put("A", new JoystickButton(this, 1));
        buttonMap.put("B", new JoystickButton(this, 2));
        buttonMap.put("X", new JoystickButton(this, 3));
        buttonMap.put("Y", new JoystickButton(this, 4));
        buttonMap.put("LBUMP", new JoystickButton(this, 5));
        buttonMap.put("RBUMP", new JoystickButton(this, 6));
        buttonMap.put("SELECT", new JoystickButton(this, 7));
        buttonMap.put("START", new JoystickButton(this, 8));

        for(int i = 1; i <= 8; i++) {
            JoystickButton button = new JoystickButton(this, i);
            button.whenPressed(new DoNothingCommand());
            button.whenReleased(new DoNothingCommand());
        }

        //initialize the POV buttons (the buttons on the dpad of the controller)
        //these constructor calls could be modified to take a 3rd argument that is their "POV value", which returns from another wpilib function, which I'm not using
        povMap.put("N", new POVButton(this, 0));
        povMap.put("NE", new POVButton(this, 45));
        povMap.put("E", new POVButton(this, 90));
        povMap.put("SE", new POVButton(this, 135));
        povMap.put("S", new POVButton(this, 180));
        povMap.put("SW", new POVButton(this, 225));
        povMap.put("W", new POVButton(this, 270));
        povMap.put("NW", new POVButton(this, 315));

        for(int i = 0; i <= 315; i += 45) {
            POVButton button = new POVButton(this, i);
            button.whenPressed(new DoNothingCommand());
            button.whenReleased(new DoNothingCommand());
        }
    }

    //assigns commands to buttons
    public void whenButtonPressed(String button, Command command) {
        buttonMap.get(button).whenPressed(command);
    }

    public void whenButtonReleased(String button, Command command) {
        buttonMap.get(button).whenReleased(command);
    }

    public void whenPOVButtonPressed(String button, Command command) {
        povMap.get(button).whenPressed(command);
    }

    public void whenPOVButtonReleased(String button, Command command) {
        povMap.get(button).whenReleased(command);
    }

    //returns whether the specified button is pressed
    public boolean isButtonPressed(String button) {
        return buttonMap.get(button).get();
    }

    public boolean isAngleMatched(String povButton) {
        return povMap.get(povButton).get();
    }

    //these get the value between -1.0 and 1.0 that represent the various joysticks
    //please note that the triggers only range between 0.0 and 1.0
    public double getRightStickX() {
        return this.getRawAxis(RIGHT_X_AXIS);
    }

    public double getRightStickY() {
        return this.getRawAxis(RIGHT_Y_AXIS);
    }

    public double getRightTrigger() {
        return this.getRawAxis(RIGHT_TRIGGER);
    }

    public double getLeftTrigger() {
        return this.getRawAxis(LEFT_TRIGGER);
    }

    public double getLeftStickX() {
        return this.getRawAxis(LEFT_X_AXIS);
    }

    public double getLeftStickY() {
        return this.getRawAxis(LEFT_Y_AXIS);
    }

    public void setTriggerTolerance(double newTolerance) {
        newTolerance = (newTolerance < 0.0) ? 0.0 : (newTolerance > 1.0) ? 1.0 : newTolerance;
        triggerTolerance = newTolerance;
    }

    public double getTriggerTolerance() {
        return triggerTolerance;
    }

    //the triggers on the controller show up as axes, but a common function is to check whether they are pressed or not
    //triggerTolerance can be set with setTriggerTolerance, and if the trigger is pressed that point, this returns true, otherwise false
    public boolean isTriggerPressed(int trigger) {
        if(trigger != 2 && trigger != 3) return false;
        return (this.getRawAxis(trigger) > triggerTolerance) ? true : false;
    }

}