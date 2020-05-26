package info.benjaminhill.depth2mesh

import info.benjaminhill.math.*
import org.apache.commons.math3.ml.clustering.DBSCANClusterer
import java.io.File
import javax.imageio.ImageIO

class AlignCloud(private val imageFile: File) {

    /** Potentially very slow */
    private val largestCluster: SimpleCloud by lazy {
        val clusterFile = File(CLUSTER_FOLDER, "${imageFile.nameWithoutExtension}.asc")
        if (!clusterFile.canRead()) {
            val rawCloud = ImageIO.read(imageFile).toPointCloud(
                minDepth = 100,
                maxDepth = 400,
                minConfidence = 0.5
            )
            val dbs = DBSCANClusterer<ClusterablePoint>(6.0, 20) // Shrug.  Doesn't seem to matter much.  Slow tho.
            val clumps = dbs.cluster(rawCloud.map { ClusterablePoint(it) }).map { it.points }
            println("Found ${clumps.size} clumps.")
            val biggestClump = clumps.maxBy { it.size }!!.map { it.point }
            biggestClump.saveToAsc(clusterFile)
            println("Created largest cluster file (${imageFile.name} to ${biggestClump.size})")
            biggestClump
        } else {
            loadPointCloudFromAsc(clusterFile)
        }
    }

    private var undistortScale = simplePointOf(1.0, 1.0, 1.0)
        set(value) {
            undistortedSimpleCloud = null
            field = value
        }

    private var undistortedSimpleCloud: SimpleCloud? = null
        get() = field ?: largestCluster.map { it * undistortScale }.also { field = it }
        set(value) {
            preppedSimpleCloud = null
            field = value
        }

    var decimateDist = 3.0
        set(value) {
            preppedSimpleCloud = null
            field = value
        }

    private var preppedSimpleCloud: SimpleCloud? = null
        get() = field ?: undistortedSimpleCloud!!.decimate(decimateDist).toCentered().also { field = it }
        set(value) {
            transformedSimpleCloud = null
            field = value
        }

    private var transform = identityMatrixOf(4)
        set(value) {
            transformedSimpleCloud = null
            field = value
        }

    var transformedSimpleCloud: SimpleCloud? = null
        get() = field ?: preppedSimpleCloud!!.map { transform.transform(it) }.toMutableList().also { field = it }

    /** Updates the transform to get closer to the other cloud */
    fun alignTo(other: AlignCloud) {
        val neighbors = transformedSimpleCloud!!.getNearestNeighbors(other.transformedSimpleCloud!!)
        transform = alignPoints3D(neighbors.keys.toMutableList(), neighbors.values.toMutableList())
    }

    fun getNearestNeighbors(other: AlignCloud) =
        transformedSimpleCloud!!.getNearestNeighbors(other.transformedSimpleCloud!!).toList()

    /** Toss out un-matched part of the mesh.  TODO: Make better. */
    fun distanceTo(other: AlignCloud) = getNearestNeighbors(other).map { (a, b) ->
        a.distanceSq(b)
    }.filter {
        it < decimateDist * 2
    }.average()


    companion object {

        private val FACE_FOLDER = File(DATA_FOLDER, "faces")
        private val CLUSTER_FOLDER = File(DATA_FOLDER, "clusters").also { it.mkdirs() }

        fun loadAll() = FACE_FOLDER
            .walk()
            .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
            .sortedBy { it.nameWithoutExtension }
            .map { AlignCloud(it) }.toMutableList()

        fun sort(allAlignClouds: List<AlignCloud>, baseAlignCloud: AlignCloud): List<AlignCloud> =
            allAlignClouds.sortedBy { cloudToSort ->
                cloudToSort.getNearestNeighbors(baseAlignCloud).sumByDouble { (a, b) ->
                    a.distanceSq(b)
                } / cloudToSort.transformedSimpleCloud!!.size
            }
    }
}