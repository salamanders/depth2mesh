package info.benjaminhill.math

import ch.ethz.globis.phtree.PhDistanceF_L1
import ch.ethz.globis.phtree.PhTreeF
import com.google.common.collect.ArrayListMultimap
import com.google.common.collect.ListMultimap
import org.apache.commons.math4.ml.clustering.Clusterable
import org.apache.commons.math4.ml.clustering.DBSCANClusterer
import java.awt.Color
import java.awt.image.BufferedImage
import java.io.File
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

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

/** Print the line if the lineNum is a power of 2 */
private fun println2(lineNum: Int, log: () -> String) {
    if (lineNum > 4096 && (lineNum and (lineNum - 1)) == 0) {
        println(log())
    }
}

val SCAN_RES = 1/400.0 // Approximate "resolution" of the scan

fun SimpleCloud3d.largestCluster(): SimpleCloud3d {
     class ClusterablePoint(private val point: SimplePoint3d) : Clusterable {
        override fun getPoint(): DoubleArray = point
    }
    // Very slow
    val gapSize = getRange().let { sqrt(it.first.distanceSq(it.second)) } / 20.0
    val dbs = DBSCANClusterer<ClusterablePoint>(gapSize, size/100)
    val clumps = dbs.cluster(this.map { ClusterablePoint(it) }).map { it.points }
    check(clumps.isNotEmpty()) {"No clumps found for gapSize:$gapSize"}
    return clumps.maxBy { it.size }!!.map { it.point }
}


/** RBNN http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.149.2721&rep=rep1&type=pdf */
fun SimpleCloud3d.largestClusterRBNN(): SimpleCloud3d {

    val gapSize = getRange().let { sqrt(it.first.distanceSq(it.second)) } * SCAN_RES
    println("GapSize: $gapSize")
    val tree = toCentered().toTree()
    // Needs to be a list of Doubles for the Map to work
    val clusterToPoints: ListMultimap<Int, List<Double>> = ArrayListMultimap.create(size, size)
    val pointToCluster = mutableMapOf<List<Double>, Int>()
    tree.queryExtent().asSequence().map { it.toList() }.toList().let { allPoints ->
        allPoints.first().hashCode()
        clusterToPoints.putAll(0, allPoints)
        pointToCluster.putAll(allPoints.map { it to 0 })
    }
    var nextClusterId = 1

    val allPoints = tree.queryExtent()
        .asSequence()
        .map { it.toList() }
        .toList()

    allPoints
        .asSequence() // Lazy filter out previously-classified points
        .filter { pointToCluster[it] == 0 }
        .forEachIndexed { index, point ->
            // unClustered should include the current point
            val (unClustered, clustered) = tree.rangeQuery(gapSize, PhDistanceF_L1.THIS, *point.toDoubleArray())
                .asSequence()
                .map { it.toList() }
                .partition { pointToCluster[it]!! == 0 }

            println2(index) { "Matching point $index (${(100 * index.toDouble() / allPoints.size).toInt()}%) to a cluster, neighbors: ${unClustered.size} unclustered, ${clustered.size} clustered." }
            clustered
                .filter { pointToCluster[point]!! != pointToCluster[it]!! }
                .forEach { neighborInCluster ->
                    val sourceClusterId = pointToCluster[point]!!
                    val destinationClusterId = pointToCluster[neighborInCluster]!!
                    if (sourceClusterId == 0) {
                        // Just this one lonely point
                        clusterToPoints[sourceClusterId].remove(point)
                        clusterToPoints[destinationClusterId].add(point)
                        pointToCluster[point] = destinationClusterId
                    } else {
                        // Larger merge. Don't bring your friends, you want the merges to happen naturally
                        pointToCluster.putAll(clusterToPoints[sourceClusterId].map { it to destinationClusterId })
                        clusterToPoints[destinationClusterId].addAll(clusterToPoints.removeAll(sourceClusterId))
                    }
                }

            // You never found any friends.
            if (pointToCluster[point] == 0) {
                val newClusterId = nextClusterId++
                pointToCluster.putAll(unClustered.map { it to newClusterId })
                clusterToPoints[newClusterId].addAll(unClustered)
            }
        }

    println("Found ${clusterToPoints.keys().size} clusters, returning largest.")
    return clusterToPoints.asMap().entries.maxBy { it.value.size }!!.value.map { it.toDoubleArray() }
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


