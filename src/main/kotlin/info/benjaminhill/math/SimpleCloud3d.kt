package info.benjaminhill.math

import ch.ethz.globis.phtree.PhTreeF
import java.awt.Color
import java.awt.image.BufferedImage
import java.io.File
import kotlin.math.abs
import kotlin.math.max

typealias SimpleCloud3d = List<SimplePoint3d>

fun SimpleCloud3d.centroid(): SimplePoint3d {
    require(this.isNotEmpty())
    val cent = fold(SimplePoint3d(dimension)) { acc, elt ->
        acc += elt
        acc
    }
    cent *= 1.0 / size
    return cent
}

fun SimpleCloud3d.toCentered(): SimpleCloud3d {
    val centroid = centroid()
    return map {
        it - centroid
    }
}

fun SimpleCloud3d.getRange(): Pair<SimplePoint3d, SimplePoint3d> {
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
fun SimpleCloud3d.toTree(): PhTreeF<SimpleVector> = PhTreeF.create<SimpleVector>(dimension).also { tree ->
    forEach { tree.put(it, it) }
}

/**
 * Map of each point in this cloud to nearest neighbor in the other cloud
 */
fun SimpleCloud3d.getNearestNeighbors(other: SimpleCloud3d): Map<SimplePoint3d, SimplePoint3d> {
    val otherTree = other.toTree()
    return this.toTree().queryExtent().asSequence()
        .map { pt0 -> pt0 to otherTree.nearestNeighbour(1, *pt0).next()!! }
        .toMap()
}

val SimpleCloud3d.dimension: Int
    get() = 3

/**
 * Returns a new cloud with reduced points
 */
fun SimpleCloud3d.decimate(minDistance: Double): SimpleCloud3d {
    val minDistanceSq = minDistance * minDistance
    val center = centroid()
    val result = PhTreeF.create<SimplePoint3d>(dimension)
    result.put(center, center)
    // Walk outwards, adding each if they aren't crowded.
    toTree().nearestNeighbour(this.size, *center).forEach { potentialPoint: SimplePoint3d ->
        val nearest: SimplePoint3d = result.nearestNeighbour(1, *potentialPoint).next()
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
fun SimpleCloud3d.averageAlongZ(subdivisions: Int = 100, minPerBucket: Int = 2): List<SimplePoint3d> {
    val distances = getRange().let { it.first - it.second }
    val stepSize = max(abs(distances.x), abs(distances.y)) / subdivisions
    val buckets = mutableMapOf<Pair<Int, Int>, MutableList<SimplePoint3d>>()
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
fun SimpleCloud3d.saveToAsc(file: File) {
    file.printWriter().use { pw ->
        forEach { point ->
            pw.println(point.joinToString(" "))
        }
    }
    println("  Wrote output to file:${file.name}, points:${size}")
}

/**
 * Hopefully import into
 * http://fabacademy.org/archives/2014/tutorials/pointcloudToSTL.html
 */
fun loadPointCloudFromAsc(file: File): SimpleCloud3d {
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
): SimpleCloud3d = mapEach { x, y ->
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


