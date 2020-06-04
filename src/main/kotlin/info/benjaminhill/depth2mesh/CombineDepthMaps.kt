package info.benjaminhill.depth2mesh

import info.benjaminhill.math.SimplePoint3d
import info.benjaminhill.math.averageAlongZ
import info.benjaminhill.math.pretty
import info.benjaminhill.math.saveToAsc
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.runBlocking
import java.io.File


val DATA_FOLDER = File("DATA/").also { it.mkdirs() }
private val OUTPUT = File(DATA_FOLDER, "output").also { it.mkdirs() }

private fun saveStateToFiles(clouds:List<AlignableCloud>, iteration:Int? = null) {
    val allOrientedPoints = mutableListOf<SimplePoint3d>()
    clouds.forEach { allOrientedPoints.addAll(it.transformedCloud3d!!) }
    allOrientedPoints.saveToAsc(File(OUTPUT, "allmerged${iteration?.let {"_$it"}?:""}.asc"))
    val averaged = allOrientedPoints.averageAlongZ(subdivisions = 150)
    averaged.saveToAsc(File(OUTPUT, "bucketed${iteration?.let {"_$it"}?:""}.asc"))
}

fun main() = runBlocking(Dispatchers.IO) {
    OUTPUT.walk().forEach { it.delete() }

    val clouds = AlignableCloud.loadAll(File("/Users/benhill/Desktop/face"))
        //.take(3)
        .toMutableList()

    println("loaded (${clouds.size})")
    check(clouds.isNotEmpty())

    clouds.parallelStream().forEach { println("Cloud decimated size: ${it.decimatedCluster!!.size}") }

    // TODO: What is the best baseline?  First?  Middle?
    // TODO: Incremental merge, so you "build" up the target mesh with each cloud you add
    // clouds.forEach { it.decimateDist = 1.0 }

    val baselineCloud = clouds[0]
    baselineCloud.setCentered()

    println("Starting alignment.")
    repeat(50) { iteration ->
        if(iteration%10==0) {
            saveStateToFiles(clouds, iteration)
        }
        clouds.drop(1).map { cloud ->
            async {
                cloud.alignTo(baselineCloud)
            }
        }.awaitAll()
        println(" $iteration")
    }

    println(clouds.last().transform.pretty())
    saveStateToFiles(clouds)


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
