package info.benjaminhill.depth2mesh

import java.io.File
import javax.imageio.ImageIO

fun main() {
    // Load up first 2 images as a set of long points in 3d, then turn into fast trees
    var allClouds = File("DATA/faces/")
        .walk()
        .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
        .sortedBy { it.nameWithoutExtension }
        .map {
            ImageIO.read(it).toPointCloud(
                maxDistance = 600,
                minConfidence = 0.5
            )
        }
        .toList()
    println("Loaded images into ${allClouds.size} PointClouds.")

    // Collapse pairwise
    while(allClouds.size > 1) {

        allClouds = allClouds.chunked(2).mapIndexed { chunk, twoClouds->
            if(twoClouds.size==2) {
                val (baseCloud, cloudToOrient) = twoClouds
                var bestOrientedCloud = cloudToOrient
                repeat(4) { iteration ->
                    val pf = ProcrustesFit(bestOrientedCloud.getNearestNeighbors(baseCloud))
                    println("Processing cloud group $chunk, iteration $iteration: err=${pf.err}")
                    bestOrientedCloud = pf.getPTransformed()
                }
                PointCloud(mutableListOf<Point>().apply {
                    addAll(baseCloud)
                    addAll(bestOrientedCloud)
                })
            } else {
                twoClouds.first()
            }
        }
    }

    println("Success!  Total points: ${allClouds.first().size}, writing to file.")
    pointsToAsc("allmerged", allClouds.first())
}