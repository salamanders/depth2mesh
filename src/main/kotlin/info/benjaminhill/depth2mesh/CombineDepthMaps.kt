package info.benjaminhill.depth2mesh

import info.benjaminhill.math.*
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.runBlocking
import java.io.File


val DATA_FOLDER = File("DATA/").also { it.mkdirs() }

fun main() = runBlocking(Dispatchers.IO) {
    val clouds = AlignCloud.loadAll()
        //.take(3)
        .toMutableList()

    println("loaded (${clouds.size})")
    check(clouds.size > 1)

    val baselineCloud = clouds[clouds.size / 2] // Hoping the middle picture is a good baseline
    clouds.forEach { it.decimateDist = 5.0 }

    println("First alignment")
    repeat(20) { iteration ->
        clouds.map { cloud ->
            async {
                cloud.alignTo(baselineCloud)
            }
        }.awaitAll()
        println(" $iteration")
    }

    clouds.forEach { it.decimateDist = 0.0 }
    println("Fine alignment")
    repeat(20) { iteration ->
        clouds.map { ch ->
            async {
                ch.alignTo(baselineCloud)
            }
        }.awaitAll()
        println(" $iteration")
    }

    val allOrientedPoints = mutableListOf<SimplePoint>()
    clouds.forEach {
        allOrientedPoints.addAll(it.transformedSimpleCloud!!)
    }
    allOrientedPoints.saveToAsc(File(DATA_FOLDER, "allmerged.asc"))

    val averaged = allOrientedPoints.averageAlongZ(
        subdivisions=150
    )
    averaged.saveToAsc(File(DATA_FOLDER, "bucketed.asc"))

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
