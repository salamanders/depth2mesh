package info.benjaminhill.depth2mesh

import info.benjaminhill.math.*
import org.apache.commons.math3.ml.clustering.Clusterable
import org.apache.commons.math3.ml.clustering.DBSCANClusterer
import org.apache.commons.math4.linear.MatrixUtils
import org.apache.commons.math4.linear.SingularValueDecomposition
import java.io.File
import javax.imageio.ImageIO
import kotlin.math.roundToInt

class AlignableCloud(private val imageFile: File) {

    /** Potentially very slow */
    private val largestCluster: SimpleCloud by lazy {
        val clusterFile = File(CLUSTER_FOLDER, "${imageFile.nameWithoutExtension}.asc")
        if (!clusterFile.canRead()) {
            val rawCloud = ImageIO.read(imageFile).toPointCloud(
                minDepth = 100,
                maxDepth = ABSOLUTE_MAX_DEPTH.toInt(),
                minConfidence = 0.5
            )
            println("  Creating ${clusterFile.name}")
            val dbs = DBSCANClusterer<ClusterablePoint>(6.0, 20) // Shrug.  Doesn't seem to matter much.  Slow tho.
            val clumps = dbs.cluster(rawCloud.map { ClusterablePoint(it) }).map { it.points }
            val biggestClump = clumps.maxBy { it.size }!!.map { it.point }
            println("  Found ${clumps.size} clumps. Created largest cluster file (${imageFile.name} size:${biggestClump.size})")
            biggestClump.saveToAsc(clusterFile)
        }
        loadPointCloudFromAsc(clusterFile)
    }

    /**
     * Raw data of the largest cluster
     */
    private var baseCloud: SimpleCloud? = null
        get() = largestCluster
        set(value) {
            preparedCloud = null
            field = value
        }

    // TODO: Should be some percent of scale
    var decimateDist = 3.0
        set(value) {
            preparedCloud = null
            field = value
        }

    var maxDepth = ABSOLUTE_MAX_DEPTH
        set(value) {
            require(baseCloud!!.any { it.z < maxDepth }) {
                "New maxDepth of $value results in an empty cloud.  ${baseCloud!!.map { it.z }
                    .min()}..${baseCloud!!.map { it.z }.max()}"
            }
            preparedCloud = null
            field = value
        }

    /**
     * Cloud ready for alignment (decimated and centered)
     */
    private var preparedCloud: SimpleCloud? = null
        get() = field ?: baseCloud!!.filter { it.z < maxDepth }.let { depthFiltered ->
            require(depthFiltered.isNotEmpty()) {
                "maxDepth of $maxDepth results in an empty cloud. ${baseCloud!!.map { it.z }
                    .min()}..${baseCloud!!.map { it.z }.max()}"
            }
            depthFiltered.decimate(decimateDist).toCentered().also { field = it }
        }
        set(value) {
            transformedCloud = null
            field = value
        }

    var transform = identityMatrixOf(4)
        set(value) {
            transformedCloud = null
            field = value
        }

    var transformedCloud: SimpleCloud? = null
        get() = field ?: preparedCloud!!.map { transform.transform(it) }.toMutableList().also { field = it }

    /** Updates the transform to get closer to the other cloud */
    fun alignTo(other: AlignableCloud) {
        val neighbors = getNearestNeighbors(other)
        transform = alignPoints3D(neighbors.keys.toMutableList(), neighbors.values.toMutableList())
    }

    private fun getNearestNeighbors(other: AlignableCloud): Map<SimplePoint, SimplePoint> =
        transformedCloud!!.getNearestNeighbors(other.transformedCloud!!)

    /** Toss out un-matched part of the mesh.  TODO: Make better. */
    fun distanceTo(other: AlignableCloud) = getNearestNeighbors(other).map { (a, b) ->
        a.distanceSq(b)
    }.sum()

    val size: Int
        get() = transformedCloud!!.size

    companion object {
        const val ABSOLUTE_MAX_DEPTH = 400.0

        private val FACE_FOLDER = File(DATA_FOLDER, "input").also { require(it.canRead()) }
        private val CLUSTER_FOLDER = File(DATA_FOLDER, "cluster_cache").also { it.mkdirs() }

        fun loadAll() = FACE_FOLDER
            .walk()
            .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
            .sortedBy { it.nameWithoutExtension }
            .map { AlignableCloud(it) }.toMutableList()

        fun sort(allAlignableClouds: List<AlignableCloud>, baseAlignableCloud: AlignableCloud): List<AlignableCloud> =
            allAlignableClouds.sortedBy { cloudToSort ->
                cloudToSort.distanceTo(baseAlignableCloud)
            }

        internal class ClusterablePoint(private val point: SimplePoint) : Clusterable {
            override fun getPoint(): DoubleArray = point
        }
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
    val svd = SingularValueDecomposition(MatrixUtils.createRealMatrix(M))

    val U: SimpleMatrix = svd.u.data
    val V: SimpleMatrix = svd.v.data
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