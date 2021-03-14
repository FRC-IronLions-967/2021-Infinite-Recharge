package frc.robot.values;

import java.io.IOException;

public class ValuesInstance {

    public Values m_robotMap;
    public Values m_values;
    public Values m_pidValues;

    private static boolean initialized;

    private static ValuesInstance inst;

    private ValuesInstance() {
        try {
            m_values = new Values("/home/lvuser/deploy/values.properties");
            m_robotMap = new Values("/home/lvuser/deploy/robotMap.properties");
            m_pidValues = new Values("/home/lvuser/deploy/pidConst.properties");

            initialized = true;
        } catch (IOException e) {
            e.printStackTrace();

            initialized = false;
        }
    }

    public static ValuesInstance getInstance() {
        if(inst == null)  inst = new ValuesInstance();

        return inst;
    }

    public static boolean isInitialized() {
        return initialized;
    }
    
}
