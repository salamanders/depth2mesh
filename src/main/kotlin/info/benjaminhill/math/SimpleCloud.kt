package info.benjaminhill.math

import ch.ethz.globis.phtree.PhTreeF
import java.awt.Color
import java.awt.image.BufferedImage
import java.io.File
import kotlin.math.max
import kotlin.math.roundToInt

typealias SimpleCloud = List<SimplePoint>

fun SimpleCloud.centroid(): SimplePoint {
    require(this.isNotEmpty())
    val cent = fold(SimplePoint(dimension)) { acc, elt ->
        acc += elt
        acc
    }
    cent *= 1.0 / size
    return cent
}

fun SimpleCloud.toCentered(): SimpleCloud {
    val centroid = centroid()
    return map {
        it - centroid
    }
}

/**
 * Batch transform to a PhTreeF.
 * TBD if there are faster ways to batch-transform the points, or ways to batch-add to the tree.
 */
fun SimpleCloud.toTree(): PhTreeF<SimpleVector> = PhTreeF.create<SimpleVector>(dimension).also { tree ->
    forEach { tree.put(it, it) }
}

/**
 * Map of each point in this cloud to nearest neighbor in the other cloud
 */
fun SimpleCloud.getNearestNeighbors(other: SimpleCloud): Map<SimplePoint, SimplePoint> {
    val otherTree = other.toTree()
    return this.toTree().queryExtent().asSequence()
        .map { pt0 -> pt0 to otherTree.nearestNeighbour(1, *pt0).next()!! }
        .toMap()
}

val SimpleCloud.dimension: Int
    get() = 3

/**
 * Returns a new cloud with reduced points
 */
fun SimpleCloud.decimate(minDistance: Double): SimpleCloud {
    val minDistanceSq = minDistance * minDistance
    val center = centroid()
    val result = PhTreeF.create<SimplePoint>(dimension)
    result.put(center, center)
    // Walk outwards, adding each if they aren't crowded.
    toTree().nearestNeighbour(this.size, *center).forEach { potentialPoint: SimplePoint ->
        val nearest: SimplePoint = result.nearestNeighbour(1, *potentialPoint).next()
        val distSq = potentialPoint.distanceSq(nearest)
        if (distSq > minDistanceSq) {
            result.put(potentialPoint, potentialPoint)
        }
    }
    return result.queryExtent().asSequence().toMutableList()
}

/**
 * Average small areas along Z axis.  Returns grid with holes.
 */
fun SimpleCloud.averageAlongZ(subdivisions: Int = 100, minPerBucket:Int = 2): List<SimplePoint> {
    val minY = map { it.y }.min()!!.toInt()
    val maxY = map { it.y }.max()!!.toInt()
    val minX = map { it.x }.min()!!.toInt()
    val maxX = map { it.x }.max()!!.toInt()

    val stepSize = max(maxY - minY, maxX - minX) / subdivisions

    val buckets = mutableMapOf<Pair<Int, Int>, MutableList<SimplePoint>>()
    forEach { point ->
        buckets.getOrPut(Pair((point.x / stepSize).toInt(), (point.y / stepSize).toInt())) { mutableListOf() }
            .add(point)
    }
    return buckets.filter { it.value.size >= minPerBucket }.map { it.value.centroid() }
}

/**
 * Hopefully import into
 * http://fabacademy.org/archives/2014/tutorials/pointcloudToSTL.html
 */
fun SimpleCloud.saveToAsc(file: File) {
    file.printWriter().use { pw ->
        forEach { point ->
            pw.println(point.joinToString(" "))
        }
    }
    println("Wrote output to file:${file.name}, points:${size}")
}

/**
 * Hopefully import into
 * http://fabacademy.org/archives/2014/tutorials/pointcloudToSTL.html
 */
fun loadPointCloudFromAsc(file: File): SimpleCloud {
    require(file.canRead())
    return file.readLines().map { line ->
        line.split(" ").map { it.toDouble() }.toDoubleArray()
    }
}


/**
 * Hacky depth map in a PNG - depth=red*256+green, confidence=blue
 */
internal fun BufferedImage.getDepthConfidence(x: Int, y: Int): Pair<Int, Double> {
    val rgb = Color(getRGB(x, y))
    val dist = (rgb.red * 256) + rgb.green
    val confidence = rgb.blue / 256.0
    return Pair(dist, confidence)
}

internal fun BufferedImage.toPointCloud(
    minDepth: Int = 0,
    maxDepth: Int = Int.MAX_VALUE,
    minConfidence: Double = 0.0
): SimpleCloud = mapEach { x, y ->
    val (depth, confidence) = this.getDepthConfidence(x, y)
    if (depth in minDepth..maxDepth && confidence >= minConfidence) {
        val xf = (x - 320) / 400.0
        val yf = (y - 240) / 400.0
        simplePointOf(xf * depth, yf * depth, depth.toDouble())
    } else {
        null
    }
}.flatten().filterNotNull().toMutableList()


/** Helper to "do stuff" to each pixel */
internal inline fun <reified T> BufferedImage.mapEach(fn: (x: Int, y: Int) -> T) = Array(this.width) { x ->
    Array(this.height) { y ->
        fn(x, y)
    }
}


/**
 * returns T, an n+1 by n+1 homogeneous transform points of dimension n
 * from list pointsA to list pointsB using method described in
 * "Least Squares Estimation of Transformation Parameters Between Two Point Patterns"
 * by Shinji Umeyana
 *
 * Algorithm overiew:
 *   a. Compute centroids of both lists, and center pointsA, pointsB at origin
 *   b. compute M\[n]\[n] = \Sum b_i * a_i^t
 *   c. given M = UDV^t via singular value decomposition, compute rotation
 *      via R = USV^t where S = diag(1,1 .. 1, det(U)*det(V));
 *   d. result computed by compounding differences in centroid and rotation matrix
 */
fun alignPoints3D(
    pointsA: SimpleCloud,
    pointsB: SimpleCloud
): Transform {
    require(pointsA.size == pointsB.size)
    require(pointsA.isNotEmpty())

    val cloudSize = pointsA.size
    val dimensions: Int = pointsA[0].size
    require(dimensions in 2..3) { "Dimension out of range: $dimensions" }

    val aCent = pointsA.centroid()
    val bCent = pointsB.centroid()

    // Now compute M = \Sig (x'_b) (x'_a)^t
    val M = squareMatrixOf(dimensions)

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
    val svd = SingularValueDecomposition(M)
    val U: SimpleMatrix = svd.u.deepCopy()
    val V: SimpleMatrix = svd.v.deepCopy()
    // compute sign, if -1, we need to swap the sign of S
    val det = U.det() * V.det()
    val S = identityMatrixOf(dimensions)
    S[dimensions - 1][dimensions - 1] = det.roundToInt().toDouble() // swap sign if necessary
    val R = matrixABCt(U, S, V)

    // Compute T = matrixABC(O2B, Rmm, A2O); = |R t| where t = bCent  + R*(-aCent)
    val t = bCent + matrixAB(R, aCent * -1.0)
    val T = squareMatrixOf(dimensions + 1)
    for (i in 0 until dimensions) System.arraycopy(R[i], 0, T[i], 0, dimensions)
    for (i in 0 until dimensions) T[i][dimensions] = t[i]
    T[dimensions][dimensions] = 1.0
    return T
}