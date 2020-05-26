package info.benjaminhill.math

typealias Transform = SimpleMatrix

/**
 * computes R p + t when T = |R t| and len(p) = len(t) |0 1|
 */
fun Transform.transform(vec: SimpleVector): SimpleVector {
    require(isSquare())
    require(vec.size + 1 == size)
    val result = SimpleVector(vec.size)
    for (i in vec.indices)
        for (j in vec.indices)
            result[i] += this[i][j] * vec[j]
    for (i in vec.indices)
        result[i] += this[i][vec.size]
    return result
}