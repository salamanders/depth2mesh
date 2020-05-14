package info.benjaminhill.depth2mesh

import java.io.File
import javax.imageio.ImageIO

fun main() {
    // Load up first 2 images as a set of long points in 3d, then turn into fast trees
    val allClouds = File("DATA/faces/")
        .walk()
        .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
        .sortedBy { it.nameWithoutExtension }
        .take(2)
        .map {
            ImageIO.read(it).toPointCloud(
                maxDistance = 400,
                minConfidence = 0.5
            )
        }
        .toList()
    println("Loaded images into ${allClouds.size} PointClouds.")

    val baseCloud = allClouds.first()
    val merged = mutableListOf<Point>()
    // Ok that we repeat the first cloud
    allClouds.forEachIndexed { index, otherCloud ->
        val pf = ProcrustesFit(baseCloud.getNearestNeighbors(otherCloud))
        println("Processing cloud $index: err=${pf.err}")
        merged.addAll(pf.getPTransformed())
    }

    println("Success!  Total points: ${merged.size}, writing to file.")
    pointsToAsc("allmerged", merged)
}