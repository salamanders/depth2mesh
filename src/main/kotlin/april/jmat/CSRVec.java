package april.jmat;

/**
 * Sparse vector implementation using a column-sorted row.
 **/
public class CSRVec extends Vec {
    static final int MIN_SIZE = 16;
    public final int length; // logical length of the vector
    public int[] indices;
    public double[] values;
    public int nz;     // how many elements of indices/values are valid?
    int lastGetIdx, lastSetIdx;

    public CSRVec(int length) {
        this(length, MIN_SIZE);
    }

    public CSRVec(int length, int capacity) {
        this.length = length;
        this.nz = 0;
        this.indices = new int[capacity];
        this.values = new double[capacity];
    }

    public final Vec copy() {
        CSRVec X = new CSRVec(length, indices.length);
        System.arraycopy(indices, 0, X.indices, 0, nz);
        System.arraycopy(values, 0, X.values, 0, nz);
        X.nz = nz;

        return X;
    }

    public final double[] copyArray() {
        return getDoubles();
    }

    private double[] getDoubles() {
        double[] v = new double[length];
        for (int i = 0; i < nz; i++)
            v[indices[i]] = values[i];
        return v;
    }

    // make the ith element correspond to the (idx,v) tuple, moving
    // anything after it as necessary. Position 'i' must be the
    // correct position for idx. 'idx' must not already be in the vec.
    final void insert(int i, int idx, double v) {
        if (v == 0)
            return;

        if (nz == indices.length)
            grow();

        System.arraycopy(indices, i, indices, i + 1, nz - i);
        System.arraycopy(values, i, values, i + 1, nz - i);

        indices[i] = idx;
        values[i] = v;
        nz++;
    }


    final void grow() {
        int newcapacity = indices.length * 2;
        while (newcapacity < 0)
            newcapacity *= 2;

        int[] newindices = new int[newcapacity];
        double[] newvalues = new double[newcapacity];

        System.arraycopy(indices, 0, newindices, 0, nz);
        System.arraycopy(values, 0, newvalues, 0, nz);

        indices = newindices;
        values = newvalues;
    }

    public final int getNz() {
        return nz;
    }

    // not thread safe. Maintain a cursor to help consecutive
    // accesses.
    public final double get(int idx) {
        if (nz == 0)
            return 0;

        if (lastGetIdx >= nz || lastGetIdx < 0)
            lastGetIdx = nz / 2;

        if (indices[lastGetIdx] < idx) {
            // search up
            while (lastGetIdx + 1 < nz && indices[lastGetIdx + 1] <= idx)
                lastGetIdx++;

        } else {
            // search down
            while (lastGetIdx - 1 >= 0 && indices[lastGetIdx - 1] >= idx)
                lastGetIdx--;

        }
        if (indices[lastGetIdx] == idx)
            return values[lastGetIdx];
        return 0;
    }

    public final void set(int idx, double v) {
        if (lastSetIdx < 0 || lastSetIdx >= nz)
            lastSetIdx = nz / 2;

        if (nz == 0) {
            if (v == 0)
                return;
            indices[0] = idx;
            values[0] = v;
            nz = 1;
            return;
        }

        if (indices[lastSetIdx] == idx) {
            values[lastSetIdx] = v;
            return;
        }

        // search.
        if (indices[lastSetIdx] < idx) {
            // search up
            while (lastSetIdx + 1 < nz && indices[lastSetIdx + 1] <= idx)
                lastSetIdx++;

            if (indices[lastSetIdx] == idx) {
                values[lastSetIdx] = v;
                return;
            }
            insert(lastSetIdx + 1, idx, v);

        } else {
            // search down
            while (lastSetIdx - 1 >= 0 && indices[lastSetIdx - 1] >= idx)
                lastSetIdx--;

            if (indices[lastSetIdx] == idx) {
                values[lastSetIdx] = v;
                return;
            }

            insert(lastSetIdx, idx, v);
        }
    }

    public final double dotProduct(Vec r, int i0, int i1) {
        double acc = 0;

        if (r instanceof CSRVec) {
            CSRVec a = this;
            CSRVec b = (CSRVec) r;

            int aidx = 0, bidx = 0;
            while (aidx < a.nz && a.indices[aidx] < i0)
                aidx++;
            while (bidx < b.nz && b.indices[bidx] < i0)
                bidx++;

            while (aidx < a.nz && bidx < b.nz) {
                int ai = a.indices[aidx], bi = b.indices[bidx];

                if (ai > i1 || bi > i1)
                    break;

                if (ai == bi) {
                    acc += a.values[aidx] * b.values[bidx];
                    aidx++;
                    bidx++;
                    continue;
                }

                if (ai < bi)
                    aidx++;
                else
                    bidx++;
            }

            return acc;
        }

        // default
        int low = 0;
        while (low < nz && indices[low] < i0)
            low++;

        for (int i = low; i < nz && indices[i] <= i1; i++)
            acc += values[i] * r.get(indices[i]);

        return acc;
    }

    public final void timesEquals(double scale, int i0, int i1) {
        int low = 0;
        while (low < nz && indices[low] < i0)
            low++;

        for (int i = low; i < nz && indices[i] <= i1; i++)
            values[i] *= scale;
    }

    public final void transposeAsColumn(Matrix A, int col) {
        for (int i = 0; i < nz; i++)
            A.set(indices[i], col, values[i]);
    }

    public final void plusEquals(int idx, double v) {
        for (int i = 0; i < nz; i++) {
            if (indices[i] == idx) {
                values[i] += v;
                return;
            }
        }

        // it's a new element
        set(idx, v);
    }

}

