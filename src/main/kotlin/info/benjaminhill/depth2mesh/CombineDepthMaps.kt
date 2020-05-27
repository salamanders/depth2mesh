package info.benjaminhill.depth2mesh

import info.benjaminhill.math.SimplePoint
import info.benjaminhill.math.averageAlongZ
import info.benjaminhill.math.pretty
import info.benjaminhill.math.saveToAsc
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.runBlocking
import java.io.File
import kotlin.math.roundToInt


val DATA_FOLDER = File("DATA/").also { it.mkdirs() }

fun main() = runBlocking(Dispatchers.IO) {
    val clouds = AlignableCloud.loadAll()
        .toMutableList()

    println("loaded (${clouds.size})")
    check(clouds.isNotEmpty())

    // TODO: What is the best baseline?  First?  Middle?
    // TODO: Incremental merge, so you "build" up the target mesh with each cloud you add

    clouds.forEach { it.decimateDist = 2.0 }

    val baselineCloud = clouds[0]
    println("Starting alignment. (${(10 * clouds[0].distanceTo(clouds[1]) / clouds[0].size).roundToInt()})")
    repeat(10) { iteration ->
        clouds.drop(1).map { cloud ->
            async {
                cloud.alignTo(baselineCloud)
            }
        }.awaitAll()
        println(" $iteration (${(10 * clouds[0].distanceTo(clouds[1]) / clouds[0].size).roundToInt()})")
    }

    println(clouds.last().transform.pretty())

    val allOrientedPoints = mutableListOf<SimplePoint>()
    clouds.forEach {
        allOrientedPoints.addAll(it.transformedCloud!!)
    }
    allOrientedPoints.saveToAsc(File(DATA_FOLDER, "allmerged.asc"))

    val averaged = allOrientedPoints.averageAlongZ(
        subdivisions = 150
    )
    averaged.saveToAsc(File(DATA_FOLDER, "bucketed.asc"))

    /*
    clouds.drop(1).forEach { it.maxDepth = 250.0 }
    clouds.forEach { it.decimateDist = 7.0 }
    println("Fine alignment")
    repeat(10) { iteration ->
        clouds.map { ch ->
            async {
                ch.alignTo(baselineCloud)
            }
        }.awaitAll()
        println(" $iteration")
    }
    println(clouds.last().transform.pretty())



    */
    /*
    val mesh = mutableListOf<Facet>()
    averaged.forEachIndexed { rowNum, row ->
        row.forEachIndexed { colNum, point ->
            try {
                listOfNotNull(point, averaged[rowNum][colNum + 1], averaged[rowNum + 1][colNum + 1]).let {
                    if (it.size == 3) {
                        mesh.add(facetOf(it))
                    }
                }
            } catch (e: IndexOutOfBoundsException) {
                //ignore
            }
            try {
                listOfNotNull(point, averaged[rowNum + 1][colNum + 1], averaged[rowNum + 1][colNum]).let {
                    if (it.size == 3) {
                        mesh.add(facetOf(it))
                    }
                }
            } catch (e: IndexOutOfBoundsException) {
                //ignore
            }
        }
    }
    println("Mesh ${mesh.size}")
    mesh.saveToStl(File(DATA_FOLDER, "mesh.stl"))

     */
}
