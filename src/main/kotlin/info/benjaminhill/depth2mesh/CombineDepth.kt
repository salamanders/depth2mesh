package info.benjaminhill.depth2mesh

import ch.ethz.globis.phtree.PhTreeF
import org.apache.commons.math3.linear.ArrayRealVector
import java.io.File
import javax.imageio.ImageIO

fun main() {
    // Load up first 2 images as a set of long points in 3d, then turn into fast trees
    val (pc0, pc1) = File("DATA/faces/")
        .walk()
        .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
        .take(2)
        .map { ImageIO.read(it).toPointCloud() }
        .toList()
    println("Loaded images into Point Clouds.")

    val pf = pointCloudsToProcrustes(pc0, pc1)
    println("Success!")
    println(pf)
}

// Helpers (good for tests as well)

fun arraysToProcrustes(l0: List<DoubleArray>, l1: List<DoubleArray>): ProcrustesFit = pointCloudsToProcrustes(
    PointCloud(l0.map { ArrayRealVector(it) }),
    PointCloud(l1.map { ArrayRealVector(it) })
)

fun pointCloudsToProcrustes(pc0: PointCloud, pc1: PointCloud): ProcrustesFit =
    treesToProcrustes(pc0.toTree(), pc1.toTree())

fun treesToProcrustes(tree0: PhTreeF<DoubleArray>, tree1: PhTreeF<DoubleArray>): ProcrustesFit =
    ProcrustesFit(tree0.queryExtent().asSequence()
        .map { pt0 -> pt0 to tree1.nearestNeighbour(1, *pt0).next()!! }.toMap()
    )