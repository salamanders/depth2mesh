package april.jmat

import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*
import java.util.*

internal class AlignPoints3DTest {

    // Testing
    // Note that the result could be left handed!
    internal fun randomOrthogonalMatrix(dim: Int, r: Random): Array<DoubleArray> {
        val M = Array(dim) { DoubleArray(dim) }
        for (i in 0 until dim) for (j in 0 until dim) M[i][j] = r.nextGaussian()
        val qr = QRDecomposition(Matrix(M))
        return qr.q.copyArray()
    }

    internal fun randomVector(dim: Int, scale: Double, r: Random): DoubleArray {
        val t = DoubleArray(dim)
        for (i in 0 until dim) t[i] = r.nextDouble() * 2 * scale - scale
        return t
    }

    @Test
    fun align() {
        val r = Random(3856)
        val dimensions = intArrayOf(2, 2, 3, 3, 2)
        for (dim in dimensions) { // allow for burn in before we get to the cases we really care about (2,3 d)
            var sumDimErr = 0.0
            var maxDimErr = 0.0
            var minDimErr = Double.MAX_VALUE
            var npts = 0
            var sumTime = 0.0
            var maxTime = 0.0
            val tic = Tic()
            val ntrials = 1000
            for (trial in 0 until ntrials) {
                // generate N random vectors, translate, rotate them and check that we solve correctly
                var R:SimpleMatrix = randomOrthogonalMatrix(dim, r)
                val t = randomVector(
                    dim,
                    if (trial < 10) 0.0 else 10.0,
                    r
                ) // do ten trials with no translation
                val rDet = R.det()
                if (rDet < 0) { // make R right handed
                    val S = LinAlg.identity(dim)
                    S[dim - 1][dim - 1] = (-1).toDouble()
                    R = LinAlg.matrixAB(R, S)
                }
                val T = LinAlg.identity(dim + 1)
                for (i in 0 until dim) System.arraycopy(R[i], 0, T[i], 0, dim)
                for (j in 0 until dim) T[j][dim] = t[j]
                val start = ArrayList<DoubleArray>()
                val end = ArrayList<DoubleArray>()
                for (i in 0 until dim + trial / 4) { // make 4 set of each size, starting with the minimum
                    val s = randomVector(dim, 1.0, r)
                    val e = T.transform( s)
                    start.add(s)
                    end.add(e)
                }
                tic.tic()
                val H = alignPoints3D(start, end)
                val time = tic.toc()
                var err = 0.0
                for (i in start.indices) {
                    val s = start[i]
                    val e = end[i]
                    val ee = H.transform( s)
                    err += LinAlg.distance(ee, e)
                }
                maxDimErr = Math.max(err / start.size, maxDimErr)
                minDimErr = Math.min(err / start.size, minDimErr)
                sumDimErr += err
                npts += start.size
                maxTime = Math.max(maxTime, time)
                sumTime += time
            }
            System.out.printf(
                "Finished dim %d:   Err min/max/avg  %.15f / %.15f / %.15f    time(avg)  = %.8f  time(max) = %.8f\n",
                dim, minDimErr, maxDimErr, sumDimErr / npts, sumTime / ntrials, maxTime
            )
        }
    }
}