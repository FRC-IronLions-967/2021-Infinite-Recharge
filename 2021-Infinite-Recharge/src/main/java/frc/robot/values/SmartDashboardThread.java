/*
    This class was an experiment to see if we could get the SmartDashboard feed to run in a separate thread in order
    to prevent any blocking calls from being made inside time sensitive calls inside subsystems or commands
*/

package frc.robot.values;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardThread extends Thread {

    // hashmap that is exposed to other classes in the program
    private HashMap<String, Object> userValues;

    // hashmap for this class only, is modified before data is synced with the dashboard
    private HashMap<String, Object> networkValues;

    private ReentrantLock userValuesLock = new ReentrantLock();
    private ReentrantLock networkValuesLock = new ReentrantLock();

    private AtomicBoolean lastUpdateSuccessful = new AtomicBoolean();

    public SmartDashboardThread() {
        super("SmartDashboardThread");

        userValues = new HashMap<>();
        networkValues = new HashMap<>();
    }

    // attempts to put the value in the table to be sent to the dashboard
    // returns true if successful, or false if it cannot acquire a lock within 5 ms
    public boolean putBoolean(String key, boolean value) {
        try {
            if(networkValuesLock.tryLock(5, TimeUnit.MILLISECONDS)) {
                try {
                    networkValues.put(key, value);
                } finally {
                    networkValuesLock.unlock();
                }
                return true;
            }
            return false;
        } catch(InterruptedException e) {
            e.printStackTrace();
            return false;
        }
    }

    public boolean getBoolean(String key, boolean defaultValue) {
        userValuesLock.lock();
        // use of instanceof seems a little sketchy but it should be safe
        if(userValues.containsKey(key) && userValues.get(key) instanceof Boolean) {
            userValuesLock.unlock();
            return (Boolean) userValues.get(key);
        }

        userValuesLock.unlock();
        return defaultValue;
    }

    public boolean putString(String key, String value) {
        try {
            if(networkValuesLock.tryLock(5, TimeUnit.MILLISECONDS)) {
                try {
                    networkValues.put(key, value);
                } finally {
                    networkValuesLock.unlock();
                }
                return true;
            }
            return false;
        } catch(InterruptedException e) {
            e.printStackTrace();
            return false;
        }
    }

    public String getString(String key, String defaultValue) {
        userValuesLock.lock();
        if(userValues.containsKey(key) && userValues.get(key) instanceof String) {
            userValuesLock.unlock();
            return (String) userValues.get(key);
        }

        userValuesLock.unlock();
        return defaultValue;
    }

    public boolean putDouble(String key, double value) {
        try {
            if(networkValuesLock.tryLock(5, TimeUnit.MILLISECONDS)) {
                try {
                    networkValues.put(key, value);
                } finally {
                    networkValuesLock.unlock();
                }
                return true;
            }
            return false;
        } catch(InterruptedException e) {
            e.printStackTrace();
            return false;
        }
    }

    public double getDouble(String key, double defaultValue) {
        userValuesLock.lock();
        if(userValues.containsKey(key) && userValues.get(key) instanceof Double) {
            userValuesLock.unlock();
            return (Double) userValues.get(key);
        }

        userValuesLock.unlock();
        return defaultValue;
    }

    public boolean wasLastUpdateSuccessful() {
        return lastUpdateSuccessful.get();
    }

    private void syncDashboard() throws InterruptedException {
        if(networkValuesLock.tryLock(10, TimeUnit.MILLISECONDS)) {
            try {
                String[] keys = (String[]) networkValues.keySet().toArray();

                for(int i = 0; i < keys.length; i++) {
                    Object o = networkValues.get(keys[i]);
                    if(o instanceof Boolean) {
                        SmartDashboard.putBoolean(keys[i], (Boolean) o);
                    } else if(o instanceof Double) {
                        SmartDashboard.putNumber(keys[i], (Double) o);
                    } else if(o instanceof String) {
                        SmartDashboard.putBoolean(keys[i], (Boolean) o);
                    }
                }

            } finally {
                networkValuesLock.unlock();
            }

            lastUpdateSuccessful.set(true);

            return;

        }

        lastUpdateSuccessful.set(false);

    }

    private void syncUserValues() throws InterruptedException {
        if(userValuesLock.tryLock(10, TimeUnit.MILLISECONDS) && networkValuesLock.tryLock(10, TimeUnit.MILLISECONDS)) {

            try {
                userValues = new HashMap<>();

                for(Map.Entry<String, Object> entry : networkValues.entrySet()) {
                    userValues.put(entry.getKey(), entry.getValue());
                }

            } finally {

                userValuesLock.unlock();
                networkValuesLock.unlock();
            }
        }
    }

    @Override
    public void run() {
        try {
            syncDashboard();
            syncUserValues();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
}
