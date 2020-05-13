package info.benjaminhill.depth2mesh

import ch.ethz.globis.phtree.PhTreeF
import info.benjaminhill.depth2mesh.ProcrustesFit.Companion.prettyPrint
import info.benjaminhill.depth2mesh.ProcrustesFit.Companion.rnd
import org.apache.commons.math3.linear.ArrayRealVector
import java.io.File
import javax.imageio.ImageIO
import kotlin.math.acos

fun main() {
    // Load up first 2 images as a set of long points in 3d, then turn into fast trees
    val (tree0, tree1) = File("DATA/faces/")
        .walk()
        .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
        .take(2)
        .map { ImageIO.read(it).toPointCloud().toTree() }
        .toList()
    println("Loaded images into trees.")
    val everyNth = 1
    println("Fining nearest neighbors of every Nth($everyNth) point.")
    val allPairs = tree0.queryExtent()
        .asSequence()
        .chunked(everyNth).map { it.first() }
        .map { pt0 -> pt0 to tree1.nearestNeighbour(1, *pt0).next()!! }
        .toMap()

    println("Ready with ${allPairs.size} point pair, running Procrustes Fit")
    val pf = ProcrustesFit(allPairs)

    println("Success!")
    println("estimated alpha: ${acos(pf.orthogonalRotation.getEntry(0, 0)).rnd()}") // What is this?
    println("estimated rotation:\n${pf.orthogonalRotation.data.prettyPrint()}")
    println("estimated translation: ${pf.translation}")
    println("estimated scale: ${pf.scale.rnd()}")
    println("RMS fitting error: ${pf.err.rnd()}")
    println("euclidean fitting error: ${pf.getEuclideanError().rnd()}")
}

// Helpers (good for tests as well)

fun arraysToProcrustes(l0: List<DoubleArray>, l1: List<DoubleArray>): ProcrustesFit = pointCloudsToProcrustes(
    PointCloud(l0.map { ArrayRealVector(it) }),
    PointCloud(l1.map { ArrayRealVector(it) })
)

fun pointCloudsToProcrustes(pc0: PointCloud, pc1: PointCloud): ProcrustesFit =
    treesToProcrustes(pc0.toTree(), pc1.toTree())

fun treesToProcrustes(tree0: PhTreeF<DoubleArray>, tree1: PhTreeF<DoubleArray>): ProcrustesFit =
    ProcrustesFit(tree0.queryExtent()
        .asSequence()
        .map { pt0 -> pt0 to tree1.nearestNeighbour(1, *pt0).next()!! }
        .toMap())