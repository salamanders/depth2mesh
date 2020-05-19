package april.jmat;

/**
 * LU Decomposition
 * <p>
 * For an m-by-n matrix A with m >= n, the LU decomposition is an m-by-n
 * unit lower triangular matrix L, an n-by-n upper triangular matrix U,
 * and a permutation vector piv of length m so that A(piv,:) = L*U.
 * If m < n, then L is m-by-m and U is m-by-n.
 * <p>
 * The LU decompostion with pivoting always exists, even if the matrix is
 * singular, so the constructor will never fail.  The primary use of the
 * LU decomposition is in the solution of square systems of simultaneous
 * linear equations.  This will fail if isNonsingular() returns false.
 **/
public class LUDecomposition {
    final Matrix LU;
    final int[] piv;
    int pivsign;

    public LUDecomposition(Matrix A) {
        this(A, true);
    }

    public LUDecomposition(Matrix A, boolean autoPivot) {
        LU = A.copy();
        int m = LU.m, n = LU.n;

        pivsign = 1;
        piv = new int[m];
        for (int i = 0; i < m; i++)
            piv[i] = i;

        // Outer loop.
        for (int j = 0; j < n; j++) {

            if (n > 1000)
                System.out.printf("%d / %d\r", j, n);

            // Make a copy of the j-th column to localize references.
            Vec LUcolj = LU.getColumn(j);

            // Apply previous transformations.
            for (int i = 0; i < m; i++) {
                Vec LUrowi = LU.getRow(i);

                // Most of the time is spent in the following dot product.

                int kmax = Math.min(i, j);
                double s = LUrowi.dotProduct(LUcolj, 0, kmax - 1);

                LUcolj.plusEquals(i, -s);
                LUrowi.plusEquals(j, -s);
            }

            // Find pivot and exchange if necessary.
            int p = j;
            if (autoPivot) {
                for (int i = j + 1; i < m; i++) {
                    if (Math.abs(LUcolj.get(i)) > Math.abs(LUcolj.get(p))) {
                        p = i;
                    }
                }
            }

            if (p != j) {
                LU.swapRows(p, j);
                int k = piv[p];
                piv[p] = piv[j];
                piv[j] = k;
                pivsign = -pivsign;
            }

            // Compute multipliers.
            if (j < n && j < m && LU.get(j, j) != 0.0) {
                double LUjj = LU.get(j, j);
                for (int i = j + 1; i < m; i++) {
                    LU.timesEquals(i, j, 1.0 / LUjj);
                }
            }
        }
    }

    public Matrix getL() {
        int m = LU.m, n = LU.n;
        Matrix L = new Matrix(m, n, LU.getOptions());

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                if (i > j) {
                    L.set(i, j, LU.get(i, j));
                } else if (i == j) {
                    L.set(i, j, 1.0);
                }
            }
        }

        return L;
    }

    public Matrix getU() {
        int m = LU.m, n = LU.n;
        Matrix U = new Matrix(n, n, LU.getOptions());

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                if (i <= j) {
                    U.set(i, j, LU.get(i, j));
                }
            }
        }
        return U;
    }

}

