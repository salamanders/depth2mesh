package info.benjaminhill.depth2mesh

import ch.ethz.globis.phtree.PhTreeF
import org.apache.commons.geometry.euclidean.threed.AffineTransformMatrix3D
import org.apache.commons.geometry.euclidean.threed.Vector3D
import org.apache.commons.math4.ml.clustering.Clusterable
import org.apache.commons.math4.ml.clustering.DBSCANClusterer
import java.io.File
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/** Pose is a a rotation and translation. */
typealias Pose = AffineTransformMatrix3D

/**
 *  To change the rotation or translation, create a copy with addTransform()
 */
class OrientedCloud
    private constructor(val oriented: List<Vector3D>) {

    val size: Int = oriented.size

    /** Pair<minPoint, maxPoint> enclosing all points */
    val range: Pair<Vector3D, Vector3D> by lazy {
        val xs = oriented.map { it.x }
        val ys = oriented.map { it.y }
        val zs = oriented.map { it.z }
        Pair(
            Vector3D.of(xs.min()!!, ys.min()!!, zs.min()!!),
            Vector3D.of(xs.max()!!, ys.max()!!, zs.max()!!)
        )
    }

    /**
     * Simplest possible format, can be imported into Meshlab
     */
    fun saveToAsc(file: File) = file.printWriter().use { pw ->
        oriented.forEach { v3d ->
            pw.println("${v3d.x} ${v3d.y} ${v3d.z}")
        }
    }

    /**
     * Extracts largest cluster.
     * Very slow.  Good thing to do in parallel
     */
    fun toLargestCluster(): OrientedCloud {
        val gapBetweenClusters = range.let { sqrt(it.first.distanceSq(it.second)) } / 20.0
        val minClusterCount = size / 100
        val dbs = DBSCANClusterer<ClusterablePoint>(gapBetweenClusters, minClusterCount)
        val clumps = dbs.cluster(oriented.map { ClusterablePoint(it) }).map { it.points }
        check(clumps.isNotEmpty()) { "No clumps found for gapSize:$gapBetweenClusters" }
        return OrientedCloud(clumps.maxBy { it.size }!!.map { Vector3D.of(it.point) })
    }


    /**
     * @return New cloud with the oriented points and a new translate to center
     */
    fun toCentered() = of(oriented, Pose.createTranslation(oriented.getCentroid().negate()))

    /**
     * Lazy transform to a PhTreeF for fast nearest neighbor
     */
    private val tree: PhTreeF<Vector3D> by lazy {
        PhTreeF.create<Vector3D>(3).also { tree ->
            oriented.forEach { tree.put(it.toArray()!!, it) }
        }
    }

    /**
     * For each point in self, find closest point in other.  Used for alignment.
     */
    fun getNearestNeighbors(other: OrientedCloud): Map<Vector3D, Vector3D> {
        val result = mutableMapOf<Vector3D, Vector3D>()
        tree.queryExtent().asSequence().forEach { myPoint ->
            result[myPoint] = other.tree.nearestNeighbour(1, *myPoint.toArray()).next()!!
        }
        return result
    }

    /**
     * @return cloud with no two points closer than minDistance
     */
    fun toDecimated(minDistance: Double): OrientedCloud {
        val minDistanceSq = minDistance * minDistance
        val center = oriented.getCentroid()
        val centerDoubleArray = center.toArray()
        val result = PhTreeF.create<Vector3D>(3)
        result.put(centerDoubleArray, center)
        // Walk outwards, adding each if they aren't crowded.
        tree.nearestNeighbour(this.size, *centerDoubleArray).forEach { potentialPoint ->
            val nearest: Vector3D = result.nearestNeighbour(1, *potentialPoint.toArray()!!).next()
            val distSq = potentialPoint.distanceSq(nearest)
            if (distSq > minDistanceSq) {
                result.put(potentialPoint.toArray()!!, potentialPoint)
            }
        }
        return OrientedCloud(result.queryExtent().asSequence().toList())
    }

    /** Layer on the new pose */
    fun addTransform(addPose:Pose) = of(oriented, addPose)

    /**
     * Average small areas along Z axis.  Returns grid with holes.
     * Result keys are useful for facets.
     */
    fun toAveragedZ(subdivisions: Int = 100, minPerBucket: Int = 2): Map<Pair<Int, Int>, Vector3D> {
        val distances = range.let { it.first.subtract(it.second) }
        val stepSize = max(abs(distances.x), abs(distances.y)) / subdivisions
        val buckets = mutableMapOf<Pair<Int, Int>, MutableList<Vector3D>>()
        oriented.forEach { point ->
            buckets.getOrPut(Pair((point.x / stepSize).toInt(), (point.y / stepSize).toInt())) { mutableListOf() }
                .add(point)
        }
        return buckets
            .filter { it.value.size >= minPerBucket }
            .map { it.key to it.value.getCentroid() }
            .toMap()
    }

    companion object {
        /**
         * Construct a cloud fromo a list of points and an optional transform
         */
        fun of(base: List<Vector3D>, pose: Pose = Pose.identity()):OrientedCloud  = OrientedCloud(base.map { v3d: Vector3D ->
            pose.apply(v3d)
        })

        /**
         * Current center of everything (average location of all points)
         */
        fun List<Vector3D>.getCentroid(): Vector3D {
            require(this.isNotEmpty())
            return this.fold(Vector3D.ZERO) { acc, elt ->
                acc.add(elt)
            }.multiply(1.0 / size)
        }

        /**
         * Simplest possible format, can be imported into Meshlab
         */
        fun File.loadCloudFromAsc(): OrientedCloud {
            require(this.canRead())
            val splitter = "\\s".toRegex()
            val points = this.readLines()
                .filter { it.isNotBlank() }
                .mapNotNull { line ->
                    val parts = line.trim().split(splitter)
                    if(parts.size>=2) {
                        val (x, y, z) = parts.take(3).map { it.toDouble() }
                        Vector3D.of(x, y, z)
                    } else {
                        println("WARNING: '$line'")
                        null
                    }
                }
            return OrientedCloud(points)
        }

        fun List<OrientedCloud>.flatten(): OrientedCloud {
            val allPoints = mutableListOf<Vector3D>()
            forEach { ezc ->
                allPoints.addAll(ezc.oriented)
            }
            return OrientedCloud(allPoints)
        }

        private class ClusterablePoint(private val v3d: Vector3D) : Clusterable {
            override fun getPoint(): DoubleArray = v3d.toArray()!!
        }
    }
}

