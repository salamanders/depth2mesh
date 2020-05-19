package info.benjaminhill.depth2mesh

import april.jmat.Transform
import april.jmat.alignPoints3D
import april.jmat.transform
import org.apache.commons.math3.linear.ArrayRealVector


fun main() {
    val allClouds = getAllClouds()
    println("Loaded all clouds.")
    val allOrientedPoints = mutableListOf<DoubleArray>()
    val baseCloud = allClouds[allClouds.size / 2]

    val sortedClouds = allClouds.sortedBy {
        it.getNearestNeighbors(baseCloud).map { (a, b) ->
            a.getDistance(b)
        }.sum()
    }
    println("Sorted all clouds.")

    sortedClouds.forEachIndexed { index, cloudToOrient ->
        var bestOrientedCloud: PointCloud = cloudToOrient
        // Keep growing the options for nearest neighbors
        val targetCloud = if (allOrientedPoints.isEmpty())
            baseCloud
        else
            PointCloud(allOrientedPoints)

        repeat(20) { repeatNum ->
            println("Orienting $index.$repeatNum")

            val neighbors = bestOrientedCloud.getNearestNeighbors(targetCloud)
            val transform: Transform = alignPoints3D(
                neighbors.keys.map { it.dataRef },
                neighbors.values.map { it.dataRef }
            )
            bestOrientedCloud = PointCloud(bestOrientedCloud.map {
                transform.transform(it.dataRef)
            })
        }
        allOrientedPoints.addAll(bestOrientedCloud.map { it.toArray() })
    }

    println("Success!  Total points: ${allOrientedPoints.size}, writing to file.")
    pointsToAsc("allmerged_new", allOrientedPoints.map { ArrayRealVector(it) })
}
