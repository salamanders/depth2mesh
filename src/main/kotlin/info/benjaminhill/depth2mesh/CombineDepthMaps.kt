package info.benjaminhill.depth2mesh


import info.benjaminhill.math.*
import info.benjaminhill.utils.concurrentMap
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.asFlow
import kotlinx.coroutines.flow.flowOn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.toList
import kotlinx.coroutines.runBlocking
import org.apache.commons.math4.linear.MatrixUtils
import org.apache.commons.math4.linear.SingularValueDecomposition
import java.io.File
import javax.imageio.ImageIO
import kotlin.math.roundToInt


private val DATA_FOLDER = File("DATA/").also { it.mkdirs() }
private val INPUT_FOLDER = File(DATA_FOLDER, "inputs").also { require(it.canRead()) }
private val RESULTS_FOLDER = File(DATA_FOLDER, "results").also { it.mkdirs() }
private val RAW_POINTS_FOLDER = File(DATA_FOLDER, "raw_points_cache").also { it.mkdirs() }
private val CLUSTER_FOLDER = File(DATA_FOLDER, "cluster_cache").also { it.mkdirs() }


fun main() = runBlocking(Dispatchers.Default) {


    println("0: Clean up from last run")
    RESULTS_FOLDER.walk().filter { it.extension.toLowerCase() == "asc" }.forEach { it.delete() }

    val decimateDist = 0.00001

    val clouds = INPUT_FOLDER
        .walk()
        .filter { it.isFile && it.canRead() && listOf("png", "asc").contains(it.extension.toLowerCase()) }
        .sortedBy { it.nameWithoutExtension }
        .asFlow()
        .concurrentMap { sourceFile ->
            // STEP 1: Load sources.  (maybe images, maybe point clouds)
            val rawPointsFile = File(RAW_POINTS_FOLDER, "${sourceFile.nameWithoutExtension}.asc")
            if (!rawPointsFile.canRead()) {
                when (sourceFile.extension.toLowerCase()) {
                    "png" -> {
                        println("${sourceFile.name} image to raw point cloud ${rawPointsFile.name}")
                        val rawCloud = ImageIO.read(sourceFile).toPointCloud()
                        rawCloud.saveToAsc(rawPointsFile)
                    }
                    "asc" -> {
                        sourceFile.copyTo(rawPointsFile, true)
                        println("Copied file to ${rawPointsFile.path}")
                    }
                }
            }
            rawPointsFile
        }
        .concurrentMap { rawPointsFile ->
            // STEP 2: Get the biggest cluster
            val clusterFile = File(CLUSTER_FOLDER, "${rawPointsFile.nameWithoutExtension}.asc")
            if (!clusterFile.canRead()) {
                require(rawPointsFile.canRead() && rawPointsFile.length() > 0) { "Unable to generate cluster because missing readable non-empty ${rawPointsFile.path}" }
                val rawCloud = loadPointCloudFromAsc(rawPointsFile)
                println(" Creating clusters of ${clusterFile.name} (CPU intensive)")
                val largestCluster = rawCloud.largestCluster()
                largestCluster.saveToAsc(clusterFile)
                println(" Created largest cluster file (${clusterFile.name} size:${rawCloud.size} down to ${largestCluster.size})")
            } else {
                println("Reusing previously rendered cluster.")
            }
            clusterFile
        }
        .concurrentMap { largestClusterFile ->
            // STEP 3: Decimate as needed
            loadPointCloudFromAsc(largestClusterFile).decimate(decimateDist)
        }
        .map { pointCloud ->
            // Prep for alignment
            Pair(identityMatrixOf(4), pointCloud)
        }
        .flowOn(Dispatchers.Default)
        .toList()

    println("Loaded ${clouds.size} clouds (from the initial point sets' biggest clusters)")

    mutableListOf<SimplePoint3d>().also { allPoints ->
        clouds.forEach { (transform, baseCloud) ->
            allPoints.addAll(transform.transform(baseCloud))
        }
        allPoints.saveToAsc(File(RESULTS_FOLDER, "all_merged.asc"))
    }

    var improvedClouds = clouds

    // STEP 5: Iterative alignment using SVD
    repeat(10) { repeatNum ->
        println("Alignment $repeatNum")
        // Start aligning to previous cloud.  clouds[0] never moves.
        val nextResult = clouds.zipWithNext { (cloudTransform0, cloudOriginal0), (cloudTransform1, cloudOriginal1) ->
            // Current best transforms
            val cloud0 = cloudTransform0.transform(cloudOriginal0)
            val cloud1 = cloudTransform1.transform(cloudOriginal1)
            // Improve on the alignment
            val neighbors = cloud1.getNearestNeighbors(cloud0)
            val betterTransform = alignPoints3D(neighbors.keys.toMutableList(), neighbors.values.toMutableList())
            Pair(betterTransform, cloudOriginal1)
        }.toMutableList()
        nextResult.add(0, clouds.first())
        improvedClouds = nextResult
    }

    // Step 6: Export Point Cloud

    val allOrientedPoints = mutableListOf<SimplePoint3d>()
    improvedClouds.forEach { (transform, baseCloud) ->
        allOrientedPoints.addAll(transform.transform(baseCloud))
    }
    allOrientedPoints.saveToAsc(File(RESULTS_FOLDER, "all_merged_oriented.asc"))
    val averaged = allOrientedPoints.averageAlongZ(subdivisions = 150)
    averaged.values.toList().saveToAsc(File(RESULTS_FOLDER, "all_merged_oriented_bucketed.asc"))

    // TODO Step 7: Export STL
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

private fun alignPoints3D(
    pointsA: SimpleCloud3d,
    pointsB: SimpleCloud3d
): Transform {
    require(pointsA.size == pointsB.size)
    require(pointsA.isNotEmpty())

    val cloudSize = pointsA.size
    val dimensions: Int = pointsA[0].size
    require(dimensions in 2..3) { "Dimension out of range: $dimensions" }

    val aCent = pointsA.getCentroid()
    val bCent = pointsB.getCentroid()

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

