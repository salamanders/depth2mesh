package info.benjaminhill.depth2mesh


import info.benjaminhill.depth2mesh.OrientedCloud.Companion.loadCloudFromAsc
import info.benjaminhill.stats.pso.OptimizableFunction
import info.benjaminhill.stats.pso.PSOSwarm
import info.benjaminhill.utils.concurrentMap
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.asFlow
import kotlinx.coroutines.flow.flowOn
import kotlinx.coroutines.flow.toList
import kotlinx.coroutines.runBlocking
import org.apache.commons.geometry.euclidean.threed.Vector3D
import org.apache.commons.geometry.euclidean.threed.rotation.AxisAngleSequence
import org.apache.commons.geometry.euclidean.threed.rotation.AxisSequence
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation
import java.io.File
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.sqrt

private val DATA_FOLDER = File("DATA/").also { it.mkdirs() }
private val INPUT_FOLDER = File(DATA_FOLDER, "inputs").also { require(it.canRead()) }
private val RESULTS_FOLDER = File(DATA_FOLDER, "results").also { it.mkdirs() }
private val CLUSTER_FOLDER = File(DATA_FOLDER, "cluster_cache").also { it.mkdirs() }
private val DECIMATED_FOLDER = File(DATA_FOLDER, "decimated_cache").also { it.mkdirs() }
private const val DECIMATE_DIST = 0.0001
private const val DECIMATE_MULT = 20 // How much less used for alignment (multiplier)

val poses = mutableListOf<Pose>()

fun main() = runBlocking(Dispatchers.Default) {

    println("0: Clean up from last run")
    RESULTS_FOLDER.walk().filter { it.extension.toLowerCase() == "asc" }.forEach { it.delete() }

    println("1: Load, cluster, and decimate point clouds")

    // Split into two runs: from the middle to the last, and from the middle down to the first
    // To have successive movement without "breaking" the chain.
    val (midToFirst, midToLast) = loadClusters().let { clouds ->
        clouds.chunked(ceil(clouds.size / 2.0).toInt()).let { (a, b) ->
            Pair(a.reversed(), b)
        }
    }

    println("2A: Aligning midToFirst (${midToFirst.size}).")
    val sequenceMidDownToFirst = alignCombineTo(midToFirst.drop(0), midToFirst[0])
    println("2B: Aligning midToLast (${midToLast.size}).")
    val sequenceAll = alignCombineTo(midToLast, sequenceMidDownToFirst)
    println("3: Saving aligned clouds.")
    sequenceAll.saveToAsc(File(RESULTS_FOLDER, "all_merged_oriented.asc"))
    println("4: Saving averaged points.")
    val averagedPoints = sequenceAll.toAveragedZ(subdivisions = 150)
    OrientedCloud.of(averagedPoints.values.toList()).saveToAsc(File(RESULTS_FOLDER, "all_merged_oriented_bucketed.asc"))

    poses.forEach { pose->
        println("POSE: $pose")
    }
    // TODO Step 7: Export STL
}

fun alignCombineTo(cloud:List<OrientedCloud>, destination:OrientedCloud):OrientedCloud {
    val remainingClouds = cloud.toMutableList()
    var result = destination
    while (remainingClouds.isNotEmpty()) {
        println("Aligning, ${remainingClouds.size} remaining.")
        val nextCloud = remainingClouds[0]
        val betterPose = alignPoints3D(result, nextCloud)
        poses.add(betterPose)
        // Re-pose everyone because of momentum, but only pop the one you just used for alignment.
        for (i in remainingClouds.indices) {
            remainingClouds[i] = remainingClouds[i].addTransform(betterPose)
        }
        result = OrientedCloud.of(result.oriented + remainingClouds.removeAt(0).oriented)
    }
    return result
}


fun alignPoints3D(fixed: OrientedCloud, moving: OrientedCloud): Pose {
    // 45 degrees is a lot of movement between shots.
    // Should be able to clamp down on the axis as well.  (don't tilt your head)
    val rads = Math.toRadians(45.0)
    // 50% is for if the cloud largest cluster isn't the same and needs to be scooted around.
    val maxMovement = 0.5 * fixed.range.let {
        val dist = it.first.subtract(it.second)
        listOf(abs(dist.x), abs(dist.y), abs(dist.z)).max()!!
    }
    // XYZ rotation (in radians), and XYZ translation.  Use to build a Pose.
    // TODO: First optimize X rotation, then fine-tune.
    val parameterBounds = arrayOf(
        (-1 * rads).rangeTo(rads),
        (-1 * rads).rangeTo(rads),
        (-1 * rads).rangeTo(rads),
        (-1 * maxMovement).rangeTo(maxMovement),
        (-1 * maxMovement).rangeTo(maxMovement),
        (-1 * maxMovement).rangeTo(maxMovement),
    )

    fun DoubleArray.toPose() :Pose {
        require(this.size==6)
        val (rotX, rotY, rotZ) = take(3)
        val (tx, ty, tz) = drop(3).take(3)
        return Pose.createRotation(Vector3D.ZERO,
            QuaternionRotation.fromAxisAngleSequence(
            AxisAngleSequence.createAbsolute(AxisSequence.XYZ, rotX, rotY, rotZ)))
            .premultiply(Pose.createTranslation(tx, ty, tz))
    }

    println("Parameter bounds: ${parameterBounds.joinToString { "${it.start.round()}..${it.endInclusive.round()}" }}.")
    val lightweightMoving = moving.toDecimated(DECIMATE_DIST * DECIMATE_MULT)
    println("Aligning decimated cloud size:${lightweightMoving.size} to previous cloud.")

    val opt = OptimizableFunction(
        parameterBounds = parameterBounds
    ) { particle ->
        OrientedCloud.of(lightweightMoving.oriented, particle.toPose())
            .getNearestNeighbors(fixed).entries.sumByDouble { sqrt(it.key.distanceSq(it.value)) }
    }

    val pso = PSOSwarm(function = opt)
    pso.run()
    return pso.getBest().toPose()
}

private suspend fun loadClusters() = INPUT_FOLDER
    .walk()
    .filter { it.isFile && it.canRead() && listOf("asc", "xyz").contains(it.extension.toLowerCase()) }
    .sortedBy { it.nameWithoutExtension }
    .asFlow()
    .concurrentMap { rawPointsFile ->
        // STEP 2: Get the biggest cluster
        val clusterFile = File(CLUSTER_FOLDER, "${rawPointsFile.nameWithoutExtension}.asc")
        if (!clusterFile.canRead()) {
            require(rawPointsFile.canRead() && rawPointsFile.length() > 0) { "Unable to generate cluster because missing readable non-empty ${rawPointsFile.path}" }
            val rawCloud = rawPointsFile.loadCloudFromAsc()
            println(" Creating clusters of ${clusterFile.name} (CPU intensive)")
            val centeredCluster = rawCloud.toLargestCluster().toCentered()
            centeredCluster.saveToAsc(clusterFile)
            println(" Created largest cluster file (${clusterFile.name} size:${rawCloud.size} down to ${centeredCluster.size})")
        }
        clusterFile
    }.concurrentMap { largestClusterFile ->
        val decimatedFile = File(DECIMATED_FOLDER, "${largestClusterFile.nameWithoutExtension}.asc")
        if (!decimatedFile.canRead()) {
            val original = largestClusterFile.loadCloudFromAsc()
            val decimated = original.toDecimated(DECIMATE_DIST)
            decimated.saveToAsc(decimatedFile)
            println(" Created decimated file (${decimatedFile.name} size:${original.size} down to ${decimated.size})")

        }
        decimatedFile
    }
    .concurrentMap { readyFile ->
        readyFile.loadCloudFromAsc()
    }
    .flowOn(Dispatchers.Default)
    .toList()

fun Double.round(decimals: Int = 4): Double = "%.${decimals}f".format(this).toDouble()