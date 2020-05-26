package info.benjaminhill.math

import kotlin.random.Random

/**
 * Simplest way to store a list of doubles. Good for Operators.  Supports 3d Points
 */
typealias SimpleVector = DoubleArray

fun vectorOf(dim: Int) = DoubleArray(dim)

fun randomVectorOf(dim: Int, scale: Double): SimpleVector = SimpleVector(dim) {
    Random.nextDouble() * 2 * scale - scale
}

operator fun SimpleVector.timesAssign(scale: Double) {
    for (i in this.indices) {
        this[i] *= scale
    }
}

operator fun SimpleVector.timesAssign(other: SimpleVector) {
    require(size == other.size)
    for (i in this.indices) {
        this[i] *= other[i]
    }
}

operator fun SimpleVector.minus(other: SimpleVector) = SimpleVector(this.size) { index ->
    this[index] - other[index]
}

operator fun SimpleVector.plusAssign(other: SimpleVector) {
    require(this.size == other.size)
    for (i in this.indices) {
        this[i] += other[i]
    }
}

operator fun SimpleVector.plus(other: SimpleVector): SimpleVector {
    require(size == other.size)
    return SimpleVector(size) {
        this[it] + other[it]
    }
}

operator fun SimpleVector.times(other: SimpleVector): SimpleVector {
    require(size == other.size)
    return SimpleVector(size) {
        this[it] * other[it]
    }
}

operator fun SimpleVector.times(other: Double) = SimpleVector(this.size) {
    this[it] * other
}

fun SimpleVector.distanceSq(other: SimpleVector): Double {
    require(size == other.size)
    return indices.sumByDouble { i ->
        (this[i] - other[i]) * (this[i] - other[i])
    }
}

/**
 * dot product from [i0, i1]
 *
 * @param other
 * @param i0
 * @param i1
 * @return
 */
fun SimpleVector.dotProduct(other: SimpleVector, i0: Int, i1: Int): Double {
    if (other.size < size) return other.dotProduct(this, i0, i1)
    var acc = 0.0
    for (i in i0..i1) {
        acc += this[i] * other[i]
    }
    return acc
}

/**
 * Add the value v to specific element.
 */
fun SimpleVector.plusEquals(idx: Int, v: Double) {
    this[idx] += v
}

/**
 * Multiply the elements between indices [i0,i1] (inclusive) by v
 */
fun SimpleVector.timesEquals(scale: Double, i0: Int, i1: Int) {
    for (i in i0..i1) this[i] *= scale
}


/**
 * Insert this vector as column 'col' in matrix A. The column is
 * initially all zero. The vector should iterate through its
 * elements, calling the matrix's set method.
 */
fun SimpleVector.transposeAsColumn(A: SimpleMatrix, col: Int) {
    for (i in indices) A[i, col] = this[i]
}