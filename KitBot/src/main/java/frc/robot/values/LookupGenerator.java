package frc.robot.values;

public class LookupGenerator {
    private double deadband;  // stores the x value at which the table will begin to have non-zero values
    private double minPower;  // starting value for y after deadband is reached, set this to the minimum power you want to drive at
    private double lowCutoff;  // point on the x axis where the function switches to the second line, if using DOUBLE_LINEAR mode
    private double midPower;  // starting y value for the second line, if using DOUBLE_LINEAR mode
    private double[] lookupTable;

    public LookupGenerator() {
        this.deadband = 0;
        this.minPower = 0;
        this.lowCutoff = 0;
        this.midPower = 0;
        this.lookupTable = new double[101];
    }

    public LookupGenerator(double deadband, double minPower) {
        this.deadband = deadband;
        this.minPower = minPower;
        this.lowCutoff = 0;
        this.midPower = 0;
        this.lookupTable = new double[101];
    }

    public LookupGenerator(double deadband, double minPower, double lowCutoff, double midPower) {
        this.deadband = deadband;
        this.minPower = minPower;
        this.lowCutoff = lowCutoff;
        this.midPower = midPower;
        this.lookupTable = new double[101];
    }

    public double getDeadband() {
        return deadband;
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    public double getLowCutoff() {
        return lowCutoff;
    }

    public void setLowCutoff(double lowCutoff) {
        this.lowCutoff = lowCutoff;
    }

    public double getMidPower() {
        return midPower;
    }

    public void setMidPower(double midPower) {
        this.midPower = midPower;
    }

    public double[] getLookupTable() {
        return lookupTable;
    }

    public void calcLookup(LookupType type) {
        int tableIndex = 0;

        switch (type) {
            case DOUBLE_LINEAR:
                while(tableIndex < deadband * 100) {
                    lookupTable[tableIndex] = 0;
                    tableIndex++;
                }
                double aimSlope = (midPower - minPower)/(lowCutoff - deadband);
                while(tableIndex < lowCutoff * 100) {
                    lookupTable[tableIndex] = (aimSlope * (((double) tableIndex / 100) - deadband)) + minPower;
                    tableIndex++;
                }
                double driveSlope = (1.0 - midPower)/(1.0 - lowCutoff);
                while(tableIndex < 101) {
                    lookupTable[tableIndex] = (driveSlope * (((double) tableIndex / 100) - lowCutoff)) + midPower;
                    tableIndex++;
                }
                break;

            case QUADRATIC:
                // this assumes 1.0 is max output, and 1.0 is max input
                // using a standard form quadratic of y = a(x-h)^2 + k, we know h and k, so we need to solve for a
                // a is given by the formula (y - k)/(x - h)^2, which was obtained by solving the equation for a
                double a = (1.0 - minPower) / Math.pow((1.0 - deadband), 2);
                while(tableIndex < deadband * 100) {
                    lookupTable[tableIndex] = 0;
                    tableIndex++;
                }
                while(tableIndex < 101) {
                    // set the point in the lookup table to the quadratic value
                    lookupTable[tableIndex] = (a *Math.pow((((double) tableIndex / 100.0) - deadband), 2)) + minPower;
                    tableIndex++;
                }
                break;
        }
    }
}
