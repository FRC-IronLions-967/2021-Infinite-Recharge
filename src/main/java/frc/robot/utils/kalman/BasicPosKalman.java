/*
    This class is supposed to use a Kalman filter to track position along with x and y velocity and acceleration
    If anyone reading this actually has experience with this, I realize this is probably a complete abhorration of
    a Kalman filter, and I'm sorry

    Might end up needing to make the fields and methods of this class static as I will only run one instance and the
    CPU load might be too much for the RIO's pathetic excuse for a processor
*/

package frc.robot.utils.kalman;

import org.apache.commons.math3.linear.*;

public class BasicPosKalman {
    //state matrix
    private RealMatrix x;

    //prediction state matrix
    private RealMatrix xp;

    //coefficient matrix for the state matrix in the prediction equation
    private RealMatrix a;

    //coefficient matrix for the control matrix in the prediction equation - unused as I'm not using a control matrix
    // private Array2DRowRealMatrix b;
    //control matrix for prediction equation - unused
    // private Array2DRowRealMatrix u;

    //prediction process covariance matrix
    private RealMatrix pp;

    //process covariance matrix
    private RealMatrix p;

    //kalman gain matrix
    private RealMatrix k;

    //final measurement state matrix
    private RealMatrix y;

    //coefficient for measured state matrix in measurement equation
    private RealMatrix c;

    //coefficient matrix used for calculating kalman gain
    private RealMatrix h;

    //identity matrix
    private RealMatrix i;

    // noise matrix
    private RealMatrix q;

    //right now this configures the filter to track acceleration, velocity, and position in x and y directions
    public BasicPosKalman(RealMatrix init, RealMatrix initErr) {
        x = init;
        // u = new Matrix(new double[][] {{x.getElement(2, 0)}, {x.getElement(3, 0)}, {x.getElement(4, 0)}, {x.getElement(5, 0)}});
        xp = new Array2DRowRealMatrix(6, 1);
        //this matrix is initialized for the 50 Hz of the periodic functions
        a = new Array2DRowRealMatrix(new double[][] {{1, 0, .02, 0, .002, 0},
                                       {0, 1, 0, .02, 0, .002},
                                       {0, 0, 1, 0, .02, 0},
                                       {0, 0, 0, 1, 0, .02},
                                       {0, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 0, 1}});

        // b = new Matrix(new double[][] {{.02, 0, .002, 0},
                                    //    {0, .02, 0, .002},
                                    //    {0, 0, .02, 0},
                                    //    {0, 0, 0, .02},
                                    //    {0, 0, 0, 0},
                                    //    {0, 0, 0, 0}});
        p = initErr;
        pp = new Array2DRowRealMatrix(6, 6);
        k = new Array2DRowRealMatrix(6, 1);
        y = new Array2DRowRealMatrix(6, 1);

        //we don't have a way to directly measure position so this matrix zeroes those from the measurement
        c = new Array2DRowRealMatrix(new double[][] {{1, 0, 0, 0, 0, 0},
                                       {1, 0, 0, 0, 0, 0},
                                       {0, 0, 1, 0, 0, 0},
                                       {0, 0, 0, 1, 0, 0},
                                       {0, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 0, 1}});

        h = new Array2DRowRealMatrix(new double[][] {{1, 0, 0, 0, 0, 0},
                                       {0, 1, 0, 0, 0, 0},
                                       {0, 0, 1, 0, 0, 0},
                                       {0, 0, 0, 1, 0, 0},
                                       {0, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 0, 1}});

        i = new Array2DRowRealMatrix(new double[][] {{1, 0, 0, 0, 0, 0},
                                       {0, 1, 0, 0, 0, 0},
                                       {0, 0, 1, 0, 0, 0},
                                       {0, 0, 0, 1, 0, 0},
                                       {0, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 0, 1}});

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

    public void predict() {
        //there is a control matrix in a kalman filter, but because we are tracking the control variables i'm not using it
        //might change if this becomes an issue
        //xp = MatrixOperations.add(MatrixOperations.multiply(a, x), MatrixOperations.multiply(b, u));

        //x_p = ax + bu = w (not including w just yet - it's a noise matrix)
        // xp = MatrixOperations.multiply(a, x);
        xp = a.multiply(x);

        //p_p = a * p * a^T + q
        // pp = MatrixOperations.multiply(a, MatrixOperations.multiply(p, a.transpose()));
        pp = p.multiply(a.transpose());
        pp = a.multiply(pp);
        pp = p.add(q);
    }

    //xm is a matrix created with all of the values from the sensors
    //r is a matrix that holds the covariances of all of the sensor data
    //calculates the measured position and the kalman gain
    public void measure(RealMatrix xm, RealMatrix r) {
        // y = MatrixOperations.multiply(c, xm);
        y = c.multiply(xm);

        //i split this into three lines because it was even more unreadable the other way
        //the equation is: (pp*h)/((h*pp*h^T) + r)
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

    public void update() {
        // x = MatrixOperations.add(xp, MatrixOperations.multiply(k, MatrixOperations.subtract(y, MatrixOperations.multiply(h, x))));
        x = h.multiply(x);
        x = y.subtract(x);
        x = k.multiply(x);
        x = xp.add(x);

        // System.out.println("Final x matrix:");
        // for(int i = 0; i < x.getRowDimension(); i++) {
        //     for(int j = 0; j < x.getColumnDimension(); j++) {
        //         System.out.print(x.getData()[i][j] + " ");
        //     }
        //     System.out.println();
        // }
        // System.out.println();
        // u = new Matrix(new double[][] {{x.getElement(2, 0)}, {x.getElement(3, 0)}, {x.getElement(4, 0)}, {x.getElement(5, 0)}});
        // p = MatrixOperations.multiply(MatrixOperations.subtract(i, MatrixOperations.multiply(k, h)), p);
        RealMatrix temp = k.multiply(h);
        temp = i.subtract(p);
        p = temp.multiply(p);
        
        // System.out.println("Final p matrix:");
        // for(int i = 0; i < p.getRowDimension(); i++) {
        //     for(int j = 0; j < p.getColumnDimension(); j++) {
        //         System.out.print(p.getData()[i][j] + " ");
        //     }
        //     System.out.println();
        // }
        // System.out.println();
    }
}