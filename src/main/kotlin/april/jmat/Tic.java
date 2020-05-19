package april.jmat;

/**
 * Execution time measurement.
 **/
public class Tic {
    final long initTime;
    long startTime;

    /**
     * Includes an implicit call to tic()
     **/
    public Tic() {
        initTime = System.nanoTime();
        startTime = initTime;
    }

    /**
     * Begin measuring time from now.
     **/
    public void tic() {
        startTime = System.nanoTime();
    }

    /**
     * How much time has passed since the most recent call to tic()?
     **/
    public double toc() {
        long endTime = System.nanoTime();

        return (endTime - startTime) / 1000000000f;
    }

}
