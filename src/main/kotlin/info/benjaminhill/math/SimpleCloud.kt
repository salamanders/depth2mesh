package info.benjaminhill.math

import ch.ethz.globis.phtree.PhTreeF
import java.awt.Color
import java.awt.image.BufferedImage
import java.io.File
import kotlin.math.abs
import kotlin.math.max

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

fun SimpleCloud.getRange(): Pair<SimplePoint, SimplePoint> {
    val min = simplePointOf(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
    val max = simplePointOf(Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE)
    forEach { point ->
        for (dim in 0 until point.dimension) {
            min[dim] = min[dim].coerceAtMost(point[dim])
            max[dim] = max[dim].coerceAtLeast(point[dim])
        }
    }
    return Pair(min, max)
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
fun SimpleCloud.averageAlongZ(subdivisions: Int = 100, minPerBucket: Int = 2): List<SimplePoint> {
    val distances = getRange().let { it.first - it.second }
    val stepSize = max(abs(distances.x), abs(distances.y)) / subdivisions
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


