package org.firstinspires.ftc.teamcode.drive.localizer;

import org.apache.commons.math4.legacy.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.legacy.linear.RealMatrix;

public class Matrix extends Array2DRowRealMatrix{
    public Matrix() {
        super(3, 1);
    }
    public Matrix(int rows, int columns) {
        super(rows, columns);
    }
    public Matrix(double[] d) {
        super(d);
    }

    public Matrix rotate(double theta) {
        RealMatrix p = new Matrix(3, 3);
        p.setRow(0, new double[]{Math.cos(theta), -Math.sin(theta), 0});
        p.setRow(1, new double[]{Math.sin(theta), Math.cos(theta), 0});
        p.setRow(2, new double[]{0, 0, 1});
        return toMatrix(p.multiply(this));
    }

    public Matrix poseExp(double theta) {
        RealMatrix p = new Matrix(3, 3);
        if (theta == 0) {
            p.setRow(0, new double[]{1, 0, 0});
            p.setRow(1, new double[]{0, 1, 0});
            p.setRow(2, new double[]{0, 0, 1});
        } else {
            p.setRow(0, new double[]{Math.sin(theta) / theta, (Math.cos(theta) - 1) / theta, 0});
            p.setRow(1, new double[]{(1 - Math.cos(theta)) / theta, Math.sin(theta) / theta, 0});
            p.setRow(2, new double[]{0, 0, 1});
        }
        return toMatrix(p.multiply(this));
    }
    public static Matrix toMatrix(RealMatrix m) {
        Matrix t = new Matrix(m.getRowDimension(), m.getColumnDimension());
        for (int i = 0; i <= m.getRowDimension() - 1; i++) {
            t.setRow(i, m.getRow(i));
        }
        return t;
    }
}
