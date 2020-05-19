package april.jmat;

/**
 * QR Decomposition.
 * <p>
 * For an m-by-n matrix A with m >= n, the QR decomposition is an m-by-n
 * orthogonal matrix Q and an n-by-n upper triangular matrix R so that
 * A = Q*R.
 * <p>
 * The QR decompostion always exists, even if the matrix does not have
 * full rank, so the constructor will never fail.  The primary use of the
 * QR decomposition is in the least squares solution of nonsquare systems
 * of simultaneous linear equations.  This will fail if isFullRank()
 * returns false.
 */

public class QRDecomposition {
    final Matrix QR;
    final double[] Rdiag;

    // XXX Attrociously unoptimized for sparse case
    public QRDecomposition(Matrix A) {
        QR = A.copy();
        int m = A.getRowDimension(), n = A.getColumnDimension();
        Rdiag = new double[n];

        // Main loop.
        for (int k = 0; k < n; k++) {

            if (n > 1000)
                System.out.printf("%d / %d\r", k, n);

            // Compute 2-norm of k-th column without under/overflow.
            double nrm = 0;
            for (int i = k; i < m; i++) {
                double QRik = QR.get(i, k);
                nrm += QRik * QRik;
            }
            nrm = Math.sqrt(nrm);

            if (nrm != 0.0) {
                // Form k-th Householder vector.
                if (QR.get(k, k) < 0)
                    nrm = -nrm;

                for (int i = k; i < m; i++)
                    QR.timesEquals(i, k, 1.0 / nrm);

                QR.plusEquals(k, k, 1);

                // Apply transformation to remaining columns.
                for (int j = k + 1; j < n; j++) {
                    double s = 0.0;
                    for (int i = k; i < m; i++)
                        s += QR.get(i, k) * QR.get(i, j);

                    s = -s / QR.get(k, k);
                    for (int i = k; i < m; i++)
                        QR.plusEquals(i, j, s * QR.get(i, k));
                }
            }
            Rdiag[k] = -nrm;
        }
    }

    public Matrix getQ() {
        int m = QR.getRowDimension(), n = QR.getColumnDimension();
        Matrix Q = new Matrix(m, n);

        for (int k = n - 1; k >= 0; k--) {

            Q.set(k, k, 1);

            for (int j = k; j < n; j++) {
                if (QR.get(k, k) != 0) {
                    double s = 0.0;
                    for (int i = k; i < m; i++)
                        s += QR.get(i, k) * Q.get(i, j);

                    s = -s / QR.get(k, k);
                    for (int i = k; i < m; i++)
                        Q.plusEquals(i, j, s * QR.get(i, k));
                }
            }
        }
        return Q;
    }

}
