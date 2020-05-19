package april.jmat

/**
 * Simple linear algebra and geometry functions designed to operate
 * on doubles[].
 * <p>
 * Conventions: We assume points are projected via right multiplication. E.g.,
 * p' = Mp.
 * <p>
 * Roll: rotation around X. Pitch: rotation around Y. Yaw: rotation around Z.
 * <p>
 * Roll Pitch Yaw are evaluated in the order: roll, pitch, then yaw. I.e.,
 * rollPitchYawToMatrix(rpy) = rotateZ(rpy[2]) * rotateY(rpy[1]) * rotateX(rpy[0])
 **/

/**
 * Simplest way to store a 3d point. Enables Operators
 */
typealias PointD = DoubleArray
typealias Cluster = List<PointD>
typealias SimpleMatrix = Array<DoubleArray>
typealias Transform = SimpleMatrix

operator fun PointD.plusAssign(other: PointD) {
    require(this.size == other.size)
    for (i in this.indices) {
        this[i] += other[i]
    }
}

operator fun PointD.timesAssign(scale: Double) {
    for (i in this.indices) {
        this[i] *= scale
    }
}

operator fun PointD.minus(other: PointD) = PointD(this.size) { index ->
    this[index] - other[index]
}

fun Cluster.centroid(): PointD {
    require(this.isNotEmpty())
    val dim = first().size
    val cent = fold(PointD(dim)) { acc, elt ->
        acc += elt
        acc
    }
    cent *= 1.0 / size
    return cent
}

fun SimpleMatrix.isSquare() = isNotEmpty() && size == this[0].size


/**
 * Warning: This is not a terribly efficient way to compute determinants for
 * anything other than 3x3 or 2x2
 */
fun SimpleMatrix.det(): Double {
    require(isSquare())
    return when (size) {
        3 -> det33()
        2 -> det22()
        else -> {
            // Otherwise compute the LU decomposition, multiply the diagonals
            val lu = LUDecomposition(Matrix(this))
            var det = 1.0
            for (i in indices) det *= lu.l[i, i]
            for (i in indices) det *= lu.u[i, i]
            return det
        }
    }
}

/**
 * compute determinant of small matrix. For large matrices an
 * alright method is to take product of diagonals of L and U
 */
fun SimpleMatrix.det33(): Double {
    require(isSquare() && size == 3)
    return -this[0][2] * this[1][1] * this[2][0] + this[0][1] * this[1][2] * this[2][0] + this[0][2] * this[1][0] * this[2][1] - this[0][0] * this[1][2] * this[2][1] - this[0][1] * this[1][0] * this[2][2] + this[0][0] * this[1][1] * this[2][2]
}

fun SimpleMatrix.det22(): Double {
    require(isSquare() && size == 2)
    return this[0][0] * this[1][1] - this[1][0] * this[0][1]
}

/**
 * computes R p + t when T = |R t| and len(p) = len(t) |0 1|
 */
fun Transform.transform(p: PointD): PointD {
    require(isSquare())
    require(p.size + 1 == size)
    val o = PointD(p.size)
    for (i in p.indices)
        for (j in p.indices)
            o[i] += this[i][j] * p[j]
    for (i in p.indices)
        o[i] += this[i][p.size]
    return o
}