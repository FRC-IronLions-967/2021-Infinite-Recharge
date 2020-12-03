/*
    This class is supposed to use a Kalman filter to track position along with x and y velocity and acceleration
    If anyone reading this actually has experience with this, I realize this is probably a complete abhorration of
    a Kalman filter, and I'm sorry

    Might end up needing to make the fields and methods of this class static as I will only run one instance and the
    CPU load might be too much for the RIO's pathetic excuse for a processor

    Update: This actually seems to work despite being arguably bad code.  Not sure how that ended up happening.  It's
    probably pretty unreadable but hopefully it's not too bad.
*/

package frc.robot.utils.kalman;

import org.apache.commons.math3.linear.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BasicPosKalman {
    // state matrix
    private RealMatrix x;

    // prediction state matrix
    private RealMatrix xp;

    // coefficient matrix for the state matrix in the prediction equation
    private RealMatrix a;

    // coefficient matrix for the control matrix in the prediction equation - unused as I'm not using a control matrix
    // private Array2DRowRealMatrix b;
    // control matrix for prediction equation - unused
    // private Array2DRowRealMatrix u;

    // prediction process covariance matrix
    private RealMatrix pp;

    // process covariance matrix
    private RealMatrix p;

    // kalman gain matrix
    private RealMatrix k;

    // final measurement state matrix
    private RealMatrix y;

    // coefficient for measured state matrix in measurement equation
    // private RealMatrix c;

    // coefficient matrix used for calculating kalman gain
    private RealMatrix h;

    // identity matrix
    // private RealMatrix i;

    // noise matrix
    private RealMatrix q;

    // right now this configures the filter to track acceleration, velocity, and position in x and y directions
    public BasicPosKalman(RealMatrix init, RealMatrix initErr) {
        x = init;

        xp = new Array2DRowRealMatrix(6, 1);

        // this matrix is initialized for the 50 Hz of the periodic functions used elsewhere in the program
        a = new Array2DRowRealMatrix(new double[][] {{1, 0, .02, 0, .002, 0},
                                       {0, 1, 0, .02, 0, .002},
                                       {0, 0, 1, 0, .02, 0},
                                       {0, 0, 0, 1, 0, .02},
                                       {0, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 0, 1}});

        p = initErr;
        pp = new Array2DRowRealMatrix(6, 6);
        k = new Array2DRowRealMatrix(6, 1);
        y = new Array2DRowRealMatrix(6, 1);

        // since c, h, and i have the same value and are never modified, I'm only going to use one of them
        // this may have to be changed if different coefficient matrices are used

        // c = new Array2DRowRealMatrix(new double[][] {{1, 0, 0, 0, 0, 0},
        //                                {0, 1, 0, 0, 0, 0},
        //                                {0, 0, 1, 0, 0, 0},
        //                                {0, 0, 0, 1, 0, 0},
        //                                {0, 0, 0, 0, 1, 0},
        //                                {0, 0, 0, 0, 0, 1}});

        h = new Array2DRowRealMatrix(new double[][] {{1, 0, 0, 0, 0, 0},
                                       {0, 1, 0, 0, 0, 0},
                                       {0, 0, 1, 0, 0, 0},
                                       {0, 0, 0, 1, 0, 0},
                                       {0, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 0, 1}});

        // i = new Array2DRowRealMatrix(new double[][] {{1, 0, 0, 0, 0, 0},
        //                                {0, 1, 0, 0, 0, 0},
        //                                {0, 0, 1, 0, 0, 0},
        //                                {0, 0, 0, 1, 0, 0},
        //                                {0, 0, 0, 0, 1, 0},
        //                                {0, 0, 0, 0, 0, 1}});

        // this noise matrix prevents the covariance from becoming 0 and stopping the filter
        q = new Array2DRowRealMatrix(new double[][] {{0.02, 0, 0, 0, 0, 0},
                                                    {0, 0.02, 0, 0, 0, 0},
                                                    {0, 0, 0.02, 0, 0, 0},
                                                    {0, 0, 0, 0.02, 0, 0},
                                                    {0, 0, 0, 0, 0.02, 0},
                                                    {0, 0, 0, 0, 0, 0.02}});
    }

    public RealMatrix getX() {
        return x;
    }

    // if you (the reader) are unfamiliar with how Kalman filtering works, the Kalman filter begins every iteration by predicting where it thinks the
    // robot should be based off of the laws of kinematics, in this case xf = x0 + v0t + 1/2at^2
    // However, because we are tracking multiple values in 2 dimensions, we use the "a" matrix, which is calculated to update the position and velocity
    // using the 20 ms or 0.02 s time dictated by the 50 Hz that the periodic functions that do the calculations are run at.
    public void predict() {
        // there is a control matrix in a kalman filter, but because we are tracking the control variables i'm not using it
        // might change if this becomes an issue
        // xp = MatrixOperations.add(MatrixOperations.multiply(a, x), MatrixOperations.multiply(b, u));

        // x_p = ax + bu + w (not including w just yet - it's a noise matrix)
        // xp = MatrixOperations.multiply(a, x);
        xp = a.multiply(x);

        // p_p = (a * p * a^T) + q
        // pp = MatrixOperations.multiply(a, MatrixOperations.multiply(p, a.transpose()));
        pp = p.multiply(a.transpose());
        pp = a.multiply(pp);
        pp = p.add(q);
    }

    // xm is a matrix created with all of the values from the sensors
    // r is a matrix that holds the covariances of all of the sensor data
    // this method takes the measured values from the sensors on the robot and feeds them in.  Based off of the covariances passed in with the measurements, we
    // calculate the Kalman gain, which is a measurement of how much the filter should trust the measured values relative to its predicted values.
    public void measure(RealMatrix xm, RealMatrix r) {
        // y = c.multiply(xm);
        y = h.multiply(xm);

        // i split this into three lines because it was even more unreadable the other way
        // the equation is: (pp*h)/((h*pp*h^T) + r)
        // k = MatrixOperations.add(MatrixOperations.multiply(MatrixOperations.multiply(h, pp), h.transpose()), r);
        k = h.multiply(pp);
        k = k.multiply((Array2DRowRealMatrix) h.transpose());
        k = k.add(r);

        // Matrix inverse = new Matrix(MatrixUtils.inverse(new Matrix(k.getMat())).getData());
        RealMatrix inverse = MatrixUtils.inverse(k);

        // k = MatrixOperations.multiply(inverse, MatrixOperations.multiply(pp, h));
        k = pp.multiply(h);
        k = inverse.multiply(k);

    }

    // this updates our matrix that stores the output of the Kalman filter based off of our Kalman gain, or "k" matrix.
    // this matrix is used as the starting point in the next iteration of the Kalman filter.
    public void update() {
        x = h.multiply(x);
        x = y.subtract(x);
        x = k.multiply(x);
        x = xp.add(x);

        RealMatrix temp = k.multiply(h);
        temp = h.subtract(p);
        p = temp.multiply(p);
    }
}