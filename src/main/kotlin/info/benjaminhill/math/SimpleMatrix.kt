package info.benjaminhill.math

import java.math.BigDecimal
import java.math.RoundingMode

/**
 * Rows, then column
 */
typealias SimpleMatrix = Array<SimpleVector>

fun simpleMatrixOf(rows: Int, columns: Int = rows): SimpleMatrix = Array(rows) {
    vectorOf(columns)
}

fun squareMatrixOf(sz: Int) = simpleMatrixOf(sz)

/**
 * Return the identity matrix of size 'sz'
 */
fun identityMatrixOf(sz: Int) = squareMatrixOf(sz).also {
    for (i in 0 until sz) it[i][i] = 1.0
}

val SimpleMatrix.rowDimension: Int
    get() = size

val SimpleMatrix.columnDimension: Int
    get() = this[0].size

fun SimpleMatrix.deepCopy(): SimpleMatrix = Array(rowDimension) {
    this[it].clone()
}

fun SimpleMatrix.swapRows(a: Int, b: Int) {
    val t = this[a]
    this[a] = this[b]
    this[b] = t
}


fun SimpleMatrix.transpose(): SimpleMatrix = simpleMatrixOf(columnDimension, rowDimension).also { result ->
    for (i in 0 until rowDimension) {
        val row = this[i]
        row.transposeAsColumn(result, i)
    }
}


/**
 * Create a new DenseVec containing a copy of the column. Changes to
 * the DenseVec do NOT affect the Matrix.
 */
fun SimpleMatrix.copyColumn(col: Int): SimpleVector = SimpleVector(rowDimension) {
    this[it][col]
}

operator fun SimpleMatrix.get(r: Int, c: Int) = this[r][c]

operator fun SimpleMatrix.set(r: Int, c: Int, value: Double) {
    this[r][c] = value
}

fun SimpleMatrix.isSquare() = isNotEmpty() && size == this[0].size

/**
 * Warning: This is not a terribly efficient way to compute determinants for
 * anything other than 3x3 or 2x2
 *
 * compute determinant of small matrix. For large matrices an
 * alright method is to take product of diagonals of L and U
 */
fun SimpleMatrix.det(): Double {
    require(isSquare())
    return when (size) {
        3 -> -this[0][2] * this[1][1] * this[2][0] + this[0][1] * this[1][2] * this[2][0] + this[0][2] * this[1][0] * this[2][1] - this[0][0] * this[1][2] * this[2][1] - this[0][1] * this[1][0] * this[2][2] + this[0][0] * this[1][1] * this[2][2]
        2 -> this[0][0] * this[1][1] - this[1][0] * this[0][1]
        else -> {
            error("LUDecomposition not supported for size:$size")
            // Otherwise compute the LU decomposition, multiply the diagonals
            /*
            val lu = LUDecomposition(this)
            var det = 1.0
            for (i in indices) det *= lu.l[i, i]
            for (i in indices) det *= lu.u[i, i]
            return det

             */
        }
    }
}

fun matrixABCt(
    A: SimpleMatrix,
    B: SimpleMatrix,
    C: SimpleMatrix
): SimpleMatrix {
    return matrixAB(A, matrixABt(B, C))
}

// X = A * B
fun matrixAB(
    A: SimpleMatrix,
    B: SimpleMatrix
): SimpleMatrix {
    val n: Int = B[0].size
    val `in`: Int = A[0].size
    require(A[0].size == B.size)

    val targetX = simpleMatrixOf(A.size, n)
    for (i in A.indices) {
        for (j in 0 until n) {
            var acc = 0.0
            for (k in 0 until `in`) acc += A[i][k] * B[k][j]
            targetX[i][j] = acc
        }
    }
    return targetX
}

/**
 * X = A * B
 */
fun matrixAB(A: SimpleMatrix, B: SimpleVector): SimpleVector {
    require(A[0].size == B.size)
    val targetX = SimpleVector(A.size)
    for (i in targetX.indices) {
        var acc = 0.0
        for (k in A[0].indices) acc += A[i][k] * B[k]
        targetX[i] = acc
    }
    return targetX
}

private fun matrixABt(
    A: SimpleMatrix,
    B: SimpleMatrix
): SimpleMatrix {
    require(A[0].size == B[0].size)
    val targetX = simpleMatrixOf(A.size, B.size)
    for (i in A.indices) {
        for (j in B.indices) {
            var acc = 0.0
            for (k in A[i].indices) acc += A[i][k] * B[j][k]
            targetX[i][j] = acc
        }
    }
    return targetX
}

fun SimpleMatrix.pretty(): String = joinToString("\n") { row: SimpleVector ->
    row.map { BigDecimal(it).setScale(3, RoundingMode.HALF_EVEN) }.joinToString("\t")
}

