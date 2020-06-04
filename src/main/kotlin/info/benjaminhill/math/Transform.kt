package info.benjaminhill.math

typealias Transform = SimpleMatrix

/**
 * computes R p + t when T = |R t| and len(p) = len(t) |0 1|
 *
 * http://web.iitd.ac.in/~hegde/cad/lecture/L6_3dtrans.pdf
 * a d g p
 * b e h q
 * c f i r
 * l m n s
 * pqr = Perspective transformations
 * a:j Linear transformations – local scaling, shear, rotation / reflection
 * lmn: Translations l, m, n along x, y, and z axis
 * s: Overall scaling
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

fun Transform.transform(cloud: SimpleCloud3d): SimpleCloud3d = cloud.map { point ->
    SimpleVector(point.size).also {
        for (i in point.indices)
            for (j in point.indices)
                it[i] += this[i][j] * point[j]
        for (i in point.indices)
            it[i] += this[i][point.size]
    }
}.toList()
