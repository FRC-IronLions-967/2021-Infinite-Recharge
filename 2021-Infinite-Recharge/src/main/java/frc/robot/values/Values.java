package frc.robot.values;

import java.io.*;
import java.util.Properties;

public class Values {
    private File file;
    private Properties prop;
    private InputStream input;
    private double autoValues[];

    public Values(File valuesFile) throws IOException {
        file = valuesFile;
        prop = new Properties();
        input = new FileInputStream(file);
        prop.load(input);
        autoValues = null;
    }

    public Values(File valuesFile, String keys[]) throws IOException {
        file = valuesFile;
        prop = new Properties();
        input = new FileInputStream(file);
        prop.load(input);
        autoValues = new double[keys.length];
        for(int i = 0; i < keys.length; i++) {
            autoValues[i] = Double.parseDouble(prop.getProperty(keys[i]));
        }
    }

    public Values(String path) throws IOException {
        file = new File(path);
        prop = new Properties();
        input = new FileInputStream(file);
        prop.load(input);
        autoValues = null;
    }

    public Values(String path, String keys[]) throws IOException {
        file = new File(path);
        prop = new Properties();
        input = new FileInputStream(file);
        prop.load(input);
        autoValues = new double[keys.length];
        for(int i = 0; i < keys.length; i++) {
            autoValues[i] = Double.parseDouble(prop.getProperty(keys[i]));
        }
    }

    public File getFile() {
        return file;
    }

    public double[] getAutoValues() {
        return autoValues;
    }

    public String getValue(String key) {
        return prop.getProperty(key);
    }

    public double getDoubleValue(String key) {
        return Double.parseDouble(prop.getProperty(key));
    }

    public int getIntValue(String key) {
        return Integer.parseInt(prop.getProperty(key));
    }
}