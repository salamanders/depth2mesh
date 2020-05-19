package april.jmat

import kotlin.math.roundToInt


/**
 * returns T, an n+1 by n+1 homogeneous transform points of dimension n
 * from list pointsA to list pointsB using method described in
 * "Least Squares Estimation of Transformation Parameters Between Two Point Patterns"
 * by Shinji Umeyana
 * Algorithm overiew:
 *   a. Compute centroids of both lists, and center pointsA, pointsB at origin
 *   b. compute M[n][n] = \Sum b_i * a_i^t
 *   c. given M = UDV^t via singular value decomposition, compute rotation
 *      via R = USV^t where S = diag(1,1 .. 1, det(U)*det(V));
 *   d. result computed by compounding differences in centroid and rotation matrix
 *  For 2D points, using AlignPoints2D is about 3 times as fast for small datasets,
 *  but the cost of the SVD in this implementation is negligible for large datasets
 */
fun alignPoints3D(
    pointsA: Cluster,
    pointsB: Cluster
): Transform {
    require(pointsA.size == pointsB.size)
    require(pointsA.isNotEmpty())

    val cloudSize = pointsA.size
    val dimensions: Int = pointsA[0].size
    require(dimensions in 2..3) { "Dimension out of range: $dimensions" }

    val aCent = pointsA.centroid()
    val bCent = pointsB.centroid()

    // Now compute M = \Sig (x'_b) (x'_a)^t
    val M = Array(dimensions) { DoubleArray(dimensions) }

    for (p in 0 until cloudSize) {
        val xa = pointsA[p] - aCent
        val xb = pointsB[p] - bCent
        for (i in 0 until dimensions)
            for (j in 0 until dimensions)
                M[i][j] += xb[i] * xa[j]
    }
    // Scale by 1/n for numerical precision in next step
    for (i in 0 until dimensions)
        for (j in 0 until dimensions)
            M[i][j] /= cloudSize.toDouble()

    // compute SVD of M to get rotation
    val svd = SingularValueDecomposition(Matrix(M))
    val U: SimpleMatrix = svd.u.copyArray()
    val V: SimpleMatrix = svd.v.copyArray()
    // compute sign, if -1, we need to swap the sign of S
    val det = U.det() * V.det()
    val S = LinAlg.identity(dimensions)
    S[dimensions - 1][dimensions - 1] = det.roundToInt().toDouble() // swap sign if necessary
    val R = LinAlg.matrixABCt(U, S, V)

    // Compute T = matrixABC(O2B, Rmm, A2O); = |R t| where t = bCent  + R*(-aCent)
    val t = LinAlg.add(bCent, LinAlg.matrixAB(R, LinAlg.scale(aCent, -1.0)))
    val T = Array(dimensions + 1) { DoubleArray(dimensions + 1) }
    for (i in 0 until dimensions) System.arraycopy(R[i], 0, T[i], 0, dimensions)
    for (i in 0 until dimensions) T[i][dimensions] = t[i]
    T[dimensions][dimensions] = 1.0
    return T
}





