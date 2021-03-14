import java.io.IOException;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDThread extends Thread {

    private NetworkTableInstance inst;
    private NetworkTable visionTable;
    private NetworkTableEntry ledEntry;

    public LEDThread() {
        inst = NetworkTableInstance.getDefault();
        visionTable = inst.getTable("vision");
        ledEntry = visionTable.getEntry("ledOn");

        Runtime rt = Runtime.getRuntime();
        try {
            rt.exec("gpio mode 3 out");
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), true);
        }
    }

    @Override
    public void run() {
        for(;;) {

            ledEntry = visionTable.getEntry("ledOn");

            // System.out.println(ledEntry.getBoolean(true));

            Runtime rt = Runtime.getRuntime();
            if(ledEntry.getBoolean(true)) {
                try {
                    rt.exec("gpio write 3 1");
                } catch (IOException e) {
                    DriverStation.reportError(e.getMessage(), true);
                }
            } else {
                try {
                    rt.exec("gpio write 3 0");
                } catch (IOException e) {
                    DriverStation.reportError(e.getMessage(), true);
                }
            }
        }
    }

}