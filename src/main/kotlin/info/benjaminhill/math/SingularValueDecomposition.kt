package info.benjaminhill.math

import kotlin.math.*

/**
 * Singular Value Decomposition.
 *
 *
 * For an m-by-n matrix A with m >= n, the singular value decomposition is
 * an m-by-n orthogonal matrix U, an n-by-n diagonal matrix S, and
 * an n-by-n orthogonal matrix V so that A = U*S*V'.
 *
 * The singular values, sigma\[k] = S\[k]\[k], are ordered so that
 * sigma[0] >= sigma[1] >= ... >= sigma[n-1].
 *
 * The singular value decomposition always exists, so the constructor will
 * never fail.  The matrix condition number and the effective numerical
 * rank can be computed from this decomposition.
 *
 *
 * XXX This code has not been optimized AT ALL.
 */
class SingularValueDecomposition(A: SimpleMatrix) {
    /**
     * Octave/MATLAB:
     * (M x N) = (M x M) * (M x N) * (N x N)
     *
     *
     * JAMA:
     * (M x N) = (M x N) * (N x N) * (N x N)
     * i.e., the left singular vectors are not computed. (The missing singular values are zero).
     */
    var u: SimpleMatrix
    var v: SimpleMatrix

    // --Commented out by Inspection START (5/18/20, 1:19 PM):
    //    public SimpleMatrix getS() {
    //        return SimpleMatrix.diag(s);
    //    }
    // --Commented out by Inspection STOP (5/18/20, 1:19 PM)
    internal class SVDImpl(Arg: SimpleMatrix) {
        /**
         * Arrays for internal storage of U and V.
         */
        val U: SimpleMatrix
        val V: SimpleMatrix

        /**
         * Array for internal storage of singular values.
         *
         * @serial internal storage of singular values.
         */
        private val singularValues: DoubleArray

        companion object {
            private fun applyTransform(a: SimpleMatrix, m: Int, k: Int, j: Int) {
                var t = 0.0
                for (i in k until m) {
                    t += a[i][k] * a[i][j]
                }
                t = -t / a[k][k]
                for (i in k until m) {
                    a[i][j] += t * a[i][k]
                }
            }
        }

        init {

            // Derived from LINPACK code.
            // Initialize.
            val A = Arg.deepCopy()

            /**
             * Row and column dimensions.
             *
             * @serial row dimension.
             * @serial column dimension.
             */
            val m = Arg.rowDimension
            val n = Arg.columnDimension

            /* Apparently the failing cases are only a proper subset of (m<n),
               so let's not throw error.  Correct fix to come later?
               if (m<n) {
               throw new IllegalArgumentException("Jama SVD only works for m >= n"); }
            */
            val nu = min(m, n)
            singularValues = DoubleArray(min(m + 1, n))
            U = simpleMatrixOf(m, nu)
            V = squareMatrixOf(n)
            val e = DoubleArray(n)
            val work = DoubleArray(m)

            // Reduce A to bidiagonal form, storing the diagonal elements
            // in s and the super-diagonal elements in e.
            val nct = min(m - 1, n)
            val nrt = max(0, min(n - 2, m))
            for (k in 0 until max(nct, nrt)) {
                if (k < nct) {

                    // Compute the transformation for the k-th column and
                    // place the k-th diagonal in s[k].
                    // Compute 2-norm of k-th column without under/overflow.
                    singularValues[k] = 0.0
                    for (i in k until m) {
                        singularValues[k] =
                            hypot(
                                singularValues[k],
                                A[i][k]
                            )
                    }
                    if (singularValues[k] != 0.0) {
                        if (A[k][k] < 0.0) {
                            singularValues[k] = -singularValues[k]
                        }
                        for (i in k until m) {
                            A[i][k] /= singularValues[k]
                        }
                        A[k][k] += 1.0
                    }
                    singularValues[k] = -singularValues[k]
                }
                for (j in k + 1 until n) {
                    if ((k < nct) and (singularValues[k] != 0.0)) {
                        // Apply the transformation.
                        applyTransform(
                            A,
                            m,
                            k,
                            j
                        )
                    }

                    // Place the k-th row of A into e for the
                    // subsequent calculation of the row transformation.
                    e[j] = A[k][j]
                }
                if (k < nct) {

                    // Place the transformation in U for subsequent back
                    // multiplication.
                    for (i in k until m) {
                        U[i][k] = A[i][k]
                    }
                }
                if (k < nrt) {

                    // Compute the k-th row transformation and place the
                    // k-th super-diagonal in e[k].
                    // Compute 2-norm without under/overflow.
                    e[k] = 0.0
                    for (i in k + 1 until n) {
                        e[k] = hypot(
                            e[k],
                            e[i]
                        )
                    }
                    if (e[k] != 0.0) {
                        if (e[k + 1] < 0.0) {
                            e[k] = -e[k]
                        }
                        for (i in k + 1 until n) {
                            e[i] /= e[k]
                        }
                        e[k + 1] += 1.0
                    }
                    e[k] = -e[k]
                    if ((k + 1 < m) and (e[k] != 0.0)) {

                        // Apply the transformation.
                        for (i in k + 1 until m) {
                            work[i] = 0.0
                        }
                        for (j in k + 1 until n) {
                            for (i in k + 1 until m) {
                                work[i] += e[j] * A[i][j]
                            }
                        }
                        for (j in k + 1 until n) {
                            val t = -e[j] / e[k + 1]
                            for (i in k + 1 until m) {
                                A[i][j] += t * work[i]
                            }
                        }
                    }


                    // Place the transformation in V for subsequent
                    // back multiplication.
                    for (i in k + 1 until n) {
                        V[i][k] = e[i]
                    }
                }
            }

            // Set up the final bidiagonal matrix or order p.
            var p = min(n, m + 1)
            if (nct < n) {
                singularValues[nct] = A[nct][nct]
            }
            if (m < p) {
                singularValues[p - 1] = 0.0
            }
            if (nrt + 1 < p) {
                e[nrt] = A[nrt][p - 1]
            }
            e[p - 1] = 0.0

            // If required, generate U.
            for (j in nct until nu) {
                for (i in 0 until m) {
                    U[i][j] = 0.0
                }
                U[j][j] = 1.0
            }
            for (k in nct - 1 downTo 0) {
                if (singularValues[k] != 0.0) {
                    for (j in k + 1 until nu) {
                        applyTransform(
                            U,
                            m,
                            k,
                            j
                        )
                    }
                    for (i in k until m) {
                        U[i][k] = -U[i][k]
                    }
                    U[k][k] = 1.0 + U[k][k]
                    for (i in 0 until k - 1) {
                        U[i][k] = 0.0
                    }
                } else {
                    for (i in 0 until m) {
                        U[i][k] = 0.0
                    }
                    U[k][k] = 1.0
                }
            }


            // If required, generate V.
            for (k in n - 1 downTo 0) {
                if ((k < nrt) and (e[k] != 0.0)) {
                    for (j in k + 1 until nu) {
                        var t = 0.0
                        for (i in k + 1 until n) {
                            t += V[i][k] * V[i][j]
                        }
                        t = -t / V[k + 1][k]
                        for (i in k + 1 until n) {
                            V[i][j] += t * V[i][k]
                        }
                    }
                }
                for (i in 0 until n) {
                    V[i][k] = 0.0
                }
                V[k][k] = 1.0
            }


            // Main iteration loop for the singular values.
            val pp = p - 1
            var iter = 0
            val eps = 2.0.pow(-52.0)
            val tiny = 2.0.pow(-966.0)
            while (p > 0) {
                var k: Int
                var kase: Int

                // XXX Here is where a test for too many iterations would go.

                // This section of the program inspects for
                // negligible elements in the s and e arrays.  On
                // completion the variables kase and k are set as follows.

                // kase = 1     if s(p) and e[k-1] are negligible and k<p
                // kase = 2     if s(k) is negligible and k<p
                // kase = 3     if e[k-1] is negligible, k<p, and
                //              s(k), ..., s(p) are not negligible (qr step).
                // kase = 4     if e(p-1) is negligible (convergence).
                k = p - 2
                while (k >= -1) {
                    if (k == -1) {
                        break
                    }
                    if (abs(e[k]) <=
                        tiny + eps * (abs(singularValues[k]) + abs(singularValues[k + 1]))
                    ) {
                        e[k] = 0.0
                        break
                    }
                    k--
                }
                if (k == p - 2) {
                    kase = 4
                } else {
                    var ks: Int = p - 1
                    while (ks >= k) {
                        if (ks == k) {
                            break
                        }
                        val t = (if (ks != p) abs(e[ks]) else 0.0) +
                                if (ks != k + 1) abs(e[ks - 1]) else 0.0
                        if (abs(singularValues[ks]) <= tiny + eps * t) {
                            singularValues[ks] = 0.0
                            break
                        }
                        ks--
                    }
                    when (ks) {
                        k -> {
                            kase = 3
                        }
                        p - 1 -> {
                            kase = 1
                        }
                        else -> {
                            kase = 2
                            k = ks
                        }
                    }
                }
                k++
                when (kase) {
                    1 -> {
                        var f = e[p - 2]
                        e[p - 2] = 0.0
                        var j = p - 2
                        while (j >= k) {
                            var t =
                                hypot(
                                    singularValues[j],
                                    f
                                )
                            val cs = singularValues[j] / t
                            val sn = f / t
                            singularValues[j] = t
                            if (j != k) {
                                f = -sn * e[j - 1]
                                e[j - 1] = cs * e[j - 1]
                            }
                            var i = 0
                            while (i < n) {
                                t = cs * V[i][j] + sn * V[i][p - 1]
                                V[i][p - 1] = -sn * V[i][j] + cs * V[i][p - 1]
                                V[i][j] = t
                                i++
                            }
                            j--
                        }
                    }
                    2 -> {
                        var f = e[k - 1]
                        e[k - 1] = 0.0
                        var j = k
                        while (j < p) {
                            var t =
                                hypot(
                                    singularValues[j],
                                    f
                                )
                            val cs = singularValues[j] / t
                            val sn = f / t
                            singularValues[j] = t
                            f = -sn * e[j]
                            e[j] = cs * e[j]
                            var i = 0
                            while (i < m) {
                                t = cs * U[i][j] + sn * U[i][k - 1]
                                U[i][k - 1] = -sn * U[i][j] + cs * U[i][k - 1]
                                U[i][j] = t
                                i++
                            }
                            j++
                        }
                    }
                    3 -> {
                        // Calculate the shift.
                        val scale = max(
                            max(
                                max(
                                    max(
                                        abs(singularValues[p - 1]), abs(singularValues[p - 2])
                                    ), abs(e[p - 2])
                                ),
                                abs(singularValues[k])
                            ), abs(e[k])
                        )
                        val sp = singularValues[p - 1] / scale
                        val spm1 = singularValues[p - 2] / scale
                        val epm1 = e[p - 2] / scale
                        val sk = singularValues[k] / scale
                        val ek = e[k] / scale
                        val b = ((spm1 + sp) * (spm1 - sp) + epm1 * epm1) / 2.0
                        val c = sp * epm1 * (sp * epm1)
                        var shift = 0.0
                        if ((b != 0.0) or (c != 0.0)) {
                            shift = sqrt(b * b + c)
                            if (b < 0.0) {
                                shift = -shift
                            }
                            shift = c / (b + shift)
                        }
                        var f = (sk + sp) * (sk - sp) + shift
                        var g = sk * ek

                        // Chase zeros.
                        var j = k
                        while (j < p - 1) {
                            var t =
                                hypot(
                                    f,
                                    g
                                )
                            var cs = f / t
                            var sn = g / t
                            if (j != k) {
                                e[j - 1] = t
                            }
                            f = cs * singularValues[j] + sn * e[j]
                            e[j] = cs * e[j] - sn * singularValues[j]
                            g = sn * singularValues[j + 1]
                            singularValues[j + 1] = cs * singularValues[j + 1]
                            var i = 0
                            while (i < n) {
                                t = cs * V[i][j] + sn * V[i][j + 1]
                                V[i][j + 1] = -sn * V[i][j] + cs * V[i][j + 1]
                                V[i][j] = t
                                i++
                            }
                            t = hypot(
                                f,
                                g
                            )
                            cs = f / t
                            sn = g / t
                            singularValues[j] = t
                            f = cs * e[j] + sn * singularValues[j + 1]
                            singularValues[j + 1] = -sn * e[j] + cs * singularValues[j + 1]
                            g = sn * e[j + 1]
                            e[j + 1] = cs * e[j + 1]
                            if (j < m - 1) {
                                var i2 = 0
                                while (i2 < m) {
                                    t = cs * U[i2][j] + sn * U[i2][j + 1]
                                    U[i2][j + 1] = -sn * U[i2][j] + cs * U[i2][j + 1]
                                    U[i2][j] = t
                                    i2++
                                }
                            }
                            j++
                        }
                        e[p - 2] = f
                        iter += 1
                    }
                    4 -> {


                        // Make the singular values positive.
                        if (singularValues[k] <= 0.0) {
                            singularValues[k] = if (singularValues[k] < 0.0) -singularValues[k] else 0.0
                            var i = 0
                            while (i <= pp) {
                                V[i][k] = -V[i][k]
                                i++
                            }
                        }

                        // Order the singular values.
                        while (k < pp) {
                            if (singularValues[k] >= singularValues[k + 1]) {
                                break
                            }
                            var t = singularValues[k]
                            singularValues[k] = singularValues[k + 1]
                            singularValues[k + 1] = t
                            if (k < n - 1) {
                                var i = 0
                                while (i < n) {
                                    t = V[i][k + 1]
                                    V[i][k + 1] = V[i][k]
                                    V[i][k] = t
                                    i++
                                }
                            }
                            if (k < m - 1) {
                                var i = 0
                                while (i < m) {
                                    t = U[i][k + 1]
                                    U[i][k + 1] = U[i][k]
                                    U[i][k] = t
                                    i++
                                }
                            }
                            k++
                        }
                        iter = 0
                        p--
                    }
                }
            }
        }
    }

    companion object {
        fun hypot(a: Double, b: Double): Double {
            var r: Double
            when {
                abs(a) > abs(b) -> {
                    r = b / a
                    r = abs(a) * sqrt(1 + r * r)
                }
                b != 0.0 -> {
                    r = a / b
                    r = abs(b) * sqrt(1 + r * r)
                }
                else -> {
                    r = 0.0
                }
            }
            return r
        }
    }

    /**
     * Construct the singular value decomposition structure to access U, S and V.
     */
    init {
        val m = A.rowDimension
        val n = A.columnDimension

        if (m >= n) {
            val impl = SVDImpl(A)
            u = impl.U
            //s = impl.getSingularValues();
            v = impl.V
        } else {
            // hack to avoid buggy SVD code for m < n
            val impl = SVDImpl(A.transpose())
            u = impl.V
            //s = impl.getSingularValues();
            v = impl.U
        }
    }
}