package april.jmat;

/**
 * Matrix class supporting dense and sparse matrices.
 **/
public class Matrix {
    static final public int SPARSE = 1;
    int m, n;
    int options;
    Vec[] rows;

    public Matrix(int m, int n) {
        this(m, n, 0);
    }

    public Matrix(int m, int n, int options) {
        this.options = options;
        this.m = m;
        this.n = n;

        rows = new Vec[m];
        for (int i = 0; i < m; i++)
            rows[i] = makeVec(n);
    }

    public Matrix(double[][] A) {
        this.m = A.length;
        this.n = A[0].length;

        rows = new Vec[m];
        for (int i = 0; i < m; i++)
            rows[i] = new DenseVec(A[i]);
    }

    private Matrix() { // used by copy
    }

    final Vec makeVec(int length) {
        if ((options & SPARSE) == 0)
            return new DenseVec(length);

        return new CSRVec(length);
        //	return new HashVec(length);
    }

    public Matrix copy() {
        Matrix X = new Matrix();
        X.m = m;
        X.n = n;
        X.options = options;

        X.rows = new Vec[m];
        for (int i = 0; i < m; i++)
            X.rows[i] = rows[i].copy();

        return X;
    }

    public double[][] copyArray() {
        double[][] A = new double[m][];
        for (int i = 0; i < m; i++)
            A[i] = rows[i].copyArray();

        return A;
    }

    ////////////////////////////////////////////
    // Basic Accessors
    public int getRowDimension() {
        return m;
    }

    public int getColumnDimension() {
        return n;
    }

    public int getOptions() {
        return options;
    }

    public double get(int row, int col) {
        Vec r = rows[row];
        return r.get(col);
    }

    public void set(int row, int col, double v) {
        Vec r = rows[row];
        r.set(col, v);
    }

    /**
     * Create a new Vec containing a copy of the column. Changes to
     * the Vec do NOT affect the Matrix.
     **/
    public Vec getColumn(int col) {
        Vec res = makeVec(m);

        for (int i = 0; i < m; i++) {
            Vec r = rows[i];
            res.set(i, r.get(col));
        }

        return res;
    }

    /**
     * The Vec returned is a LIVE view of the matrix. Changes to it
     * affect the matrix. (This is NOT the case for getColumn).
     **/
    public Vec getRow(int row) {
        return rows[row];
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();

        for (int i = 0; i < m; i++) {
            sb.append("[");
            for (int j = 0; j < n; j++) {
                sb.append(String.format("%15f ", get(i, j)));
            }
            sb.append(" ]\n");
        }
        sb.append("\n");

        return sb.toString();
    }

    public void timesEquals(int i, int j, double v) {
        rows[i].timesEquals(v, j, j);
    }

    public void plusEquals(int i, int j, double v) {
        rows[i].plusEquals(j, v);
    }

    public void swapRows(int a, int b) {
        Vec t = rows[a];
        rows[a] = rows[b];
        rows[b] = t;
    }

    public Matrix transpose() {
        Matrix X = new Matrix(n, m, options);
        for (int i = 0; i < m; i++) {
            Vec row = rows[i];
            row.transposeAsColumn(X, i);
        }

        return X;
    }

}
