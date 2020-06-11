package info.benjaminhill.math

import java.math.BigDecimal
import java.math.RoundingMode

/**
 * Extension function on a SimpleVector (DoubleArray) is the simplest way to store a 2D array.
 * Good for Operators (like myMatrix[rowNum, colNum]++)
 * Rows, then column.
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
    for (rowNum in 0 until rowDimension) {
        val row = this[rowNum]
        row.transposeAsColumn(result, rowNum)
    }
}


/**
 * Create a new DenseVec containing a copy of the column. Changes to
 * the DenseVec do NOT affect the Matrix.
 */
fun SimpleMatrix.copyColumn(sourceColNum: Int): SimpleVector = SimpleVector(rowDimension) { rowNum ->
    this[rowNum][sourceColNum]
}

operator fun SimpleMatrix.get(rowNum: Int, columnNum: Int) = this[rowNum][columnNum]

operator fun SimpleMatrix.set(rowNum: Int, columnNum: Int, value: Double) {
    this[rowNum][columnNum] = value
}

fun SimpleMatrix.isSquare() = isNotEmpty() && rowDimension == columnDimension

/**
 * compute determinant of small matrix.
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
    require(A.columnDimension == B.rowDimension)

    val bNumCols: Int = B.columnDimension
    val aNumCols: Int = A.columnDimension

    val targetX = simpleMatrixOf(A.rowDimension, bNumCols)
    for (aRowNum in A.indices) {
        for (bColNum in B.indices) {
            var acc = 0.0
            for (aColNum in A.indices) {
                acc += A[aRowNum][aColNum] * B[aColNum][bColNum]
            }
            targetX[aRowNum][bColNum] = acc
        }
    }
    return targetX
}

/**
 * X = A * B
 */
fun matrixAB(A: SimpleMatrix, B: SimpleVector): SimpleVector {
    require(A.columnDimension == B.size)
    val targetX = SimpleVector(A.rowDimension)
    for (xRowNum in targetX.indices) {
        var acc = 0.0
        for (aColNum in A[0].indices) {
            acc += A[xRowNum][aColNum] * B[aColNum]
        }
        targetX[xRowNum] = acc
    }
    return targetX
}

private fun matrixABt(
    A: SimpleMatrix,
    B: SimpleMatrix
): SimpleMatrix {
    require(A.columnDimension == B.columnDimension)
    val targetX = simpleMatrixOf(A.rowDimension, B.rowDimension)
    for (aRowNum in A.indices) {
        for (bRowNum in B.indices) {
            var acc = 0.0
            for (aColNum in A[aRowNum].indices) {
                acc += A[aRowNum][aColNum] * B[bRowNum][aColNum]
            }
            targetX[aRowNum][bRowNum] = acc
        }
    }
    return targetX
}

fun SimpleMatrix.pretty(): String = joinToString("\n") { row: SimpleVector ->
    row.map { BigDecimal(it).setScale(3, RoundingMode.HALF_EVEN) }.joinToString("\t")
}

