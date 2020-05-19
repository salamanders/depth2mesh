package april.jmat;


public final class LinAlg {

    /**
     * Returns the square of v
     **/
    public static double sq(double v) {
        return v * v;
    }

    /**
     * Squared Euclidean distance using first 'len' elements
     **/
    public static double squaredDistance(double[] a, double[] b, int len) {
        double mag = 0;

        for (int i = 0; i < len; i++)
            mag += sq(b[i] - a[i]);

        return mag;
    }

    /**
     * Euclidean distance
     **/
    public static double distance(double[] a, double[] b) {
        assert (a.length == b.length);
        return Math.sqrt(squaredDistance(a, b, a.length));
    }

    public static double[] add(double[] a, double[] b) {
        return add(a, b, null);
    }

    public static double[] add(double[] a, double[] b, double[] r) {
        assert (a.length == b.length);
        if (r == null)
            r = new double[a.length];
        assert (r.length == a.length);

        for (int i = 0; i < a.length; i++)
            r[i] = a[i] + b[i];

        return r;
    }

    public static double[] copy(double[] a) {
        double[] r = new double[a.length];

        System.arraycopy(a, 0, r, 0, a.length);

        return r;
    }


    /**
     * returns v*a, allocating a new result
     **/
    public static double[] scale(double[] v, double a) {
        return scale(v, a, null);
    }

    public static double[] scale(double[] v, double a, double[] nv) {
        if (nv == null)
            nv = new double[v.length];

        for (int i = 0; i < v.length; i++)
            nv[i] = v[i] * a;
        return nv;
    }


    public static double[][] matrixABCt(double[][] A, double[][] B, double[][] C) {
        return LinAlg.matrixAB(A, LinAlg.matrixABt(B, C));
    }

    // X = A * B
    public static double[][] matrixAB(double[][] A, double[][] B) {
        return matrixAB(A, B, null);
    }

    public static double[][] matrixAB(double[][] A, double[][] B, double[][] X) {
        int m = A.length, n = B[0].length;
        int in = A[0].length;
        assert (A[0].length == B.length);
        if (X == null)
            X = new double[m][n];

        assert (X.length == m && X[0].length == n);

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                double acc = 0;
                for (int k = 0; k < in; k++)
                    acc += A[i][k] * B[k][j];
                X[i][j] = acc;
            }
        }

        return X;
    }

    /**
     * X = A * B
     **/
    public static double[] matrixAB(double[][] A, double[] B) {
        return matrixAB(A, B, null);
    }

    public static double[] matrixAB(double[][] A, double[] B, double[] X) {
        assert (A[0].length == B.length);

        if (X == null)
            X = new double[A.length];

        assert (A.length == X.length);

        for (int i = 0; i < X.length; i++) {
            double acc = 0;
            for (int k = 0; k < A[0].length; k++)
                acc += A[i][k] * B[k];
            X[i] = acc;
        }

        return X;
    }


    public static double[][] matrixABt(double[][] A, double[][] B) {
        return matrixABt(A, B, null);
    }

    public static double[][] matrixABt(double[][] A, double[][] B, double[][] X) {
        int m = A.length, n = B.length;
        int in = A[0].length;

        assert (A[0].length == B[0].length);
        if (X == null)
            X = new double[m][n];

        assert (X.length == m && X[0].length == n);

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                double acc = 0;
                for (int k = 0; k < in; k++)
                    acc += A[i][k] * B[j][k];
                X[i][j] = acc;
            }
        }

        return X;
    }


    /**
     * Return the identity matrix of size 'sz'
     **/
    public static double[][] identity(int sz) {
        double[][] M = new double[sz][sz];
        for (int i = 0; i < sz; i++)
            M[i][i] = 1;
        return M;
    }


}


