package april.jmat;

/**
 * Base type of all vectors (both sparse and dense).
 **/
public abstract class Vec {

    /**
     * How many non-zero entries are there?
     **/
    public abstract int getNz();

    /**
     * Get the element at index idx
     **/
    public abstract double get(int idx);

    /**
     * Set the element at index idx to v.
     */
    public abstract void set(int idx, double v);

    // dot product from [i0, i1]
    public abstract double dotProduct(Vec r, int i0, int i1);

    /**
     * Make a copy of the vector
     **/
    public abstract Vec copy();

    public abstract double[] copyArray();

    /**
     * Multiply the elements between indices [i0,i1] (inclusive) by
     * v
     **/
    public abstract void timesEquals(double v, int i0, int i1);

    /**
     * Add the value v to each element.
     **/
    public void plusEquals(int idx, double v) {
        set(idx, get(idx) + v);
    }

    /**
     * Insert this vector as column 'col' in matrix A. The column is
     * initially all zero. The vector should iterate through its
     * elements, calling the matrix's set method.
     **/
    public abstract void transposeAsColumn(Matrix A, int col);


}
