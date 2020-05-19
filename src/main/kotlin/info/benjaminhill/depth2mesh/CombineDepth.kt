package info.benjaminhill.depth2mesh

import kotlinx.coroutines.*


fun main() = runBlocking {
    var allClouds = getAllClouds()

    while (allClouds.size > 1) {
        val chunks = allClouds.chunked(2)
        val mergedClouds = chunks.pmapIndexed { index, twoClouds ->
            if (twoClouds.size == 2) {
                val (fixedCloud, cloudToOrient) = twoClouds
                var bestOrientedCloud = cloudToOrient
                repeat(20) { iteration ->
                    val pf = ProcrustesFit(
                        bestOrientedCloud.getNearestNeighbors(fixedCloud),
                        allowScaling = false
                    )
                    println("Processing cloud chunk $index of ${chunks.size}, iteration $iteration: err=${pf.err}")
                    bestOrientedCloud = pf.getPTransformed()
                }
                PointCloud(mutableListOf<Point>().apply {
                    addAll(fixedCloud)
                    addAll(bestOrientedCloud)
                })

            } else {
                println("Cloud gets a wildcard slot")
                twoClouds.first()
            }
        }
        allClouds = mergedClouds
    }

    println("Success!  Total points: ${allClouds.first().size}, writing to file.")
    pointsToAsc("home_merged", allClouds.first())
}

suspend fun <A, B> Iterable<A>.pmapIndexed(f: suspend (Int, A) -> B): List<B> = coroutineScope {
    mapIndexed { index, f -> async(Dispatchers.IO) { f(index, f) } }.awaitAll()
}