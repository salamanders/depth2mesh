package april.jmat;

/**
 * Default "dense" vector implementation backed by a simple array.
 **/
public class DenseVec extends Vec {
    final double[] v;

    public DenseVec(int length) {
        this.v = new double[length];
    }

    public DenseVec(double[] v) {
        this.v = v;
    }

    public final Vec copy() {
        DenseVec X = new DenseVec(v.length);
        System.arraycopy(v, 0, X.v, 0, v.length);
        return X;
    }

    public final double[] copyArray() {
        return LinAlg.copy(v);
    }

    public final int getNz() {
        return v.length;
    }

    public final double get(int idx) {
        return v[idx];
    }

    public final void set(int idx, double value) {
        v[idx] = value;
    }

    public final double dotProduct(Vec r, int i0, int i1) {
        if (r.getNz() < getNz())
            return r.dotProduct(this, i0, i1);

        double acc = 0;
        for (int i = i0; i <= i1; i++) {
            acc += v[i] * r.get(i);
        }

        return acc;
    }

    public final void timesEquals(double scale, int i0, int i1) {
        for (int i = i0; i <= i1; i++)
            v[i] *= scale;
    }

    public final void transposeAsColumn(Matrix A, int col) {
        for (int i = 0; i < v.length; i++)
            A.set(i, col, v[i]);
    }

}
