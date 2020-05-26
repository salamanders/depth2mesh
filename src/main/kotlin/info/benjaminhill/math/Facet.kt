package info.benjaminhill.math

import com.google.common.collect.BiMap
import com.google.common.collect.HashBiMap
import java.io.File
import java.nio.charset.StandardCharsets

typealias Facet = Triple<SimplePoint, SimplePoint, SimplePoint>

fun facetOf(points: List<SimplePoint>) = Triple(points[0], points[1], points[2])

fun Collection<Facet>.saveToPly(file: File) {
    file.printWriter(StandardCharsets.US_ASCII).use { pw ->
        pw.println("ply")
        pw.println("format ascii 1.0")
        pw.println("comment by Benjamin Hill")
        pw.println("comment Depth Map from smartphone")

        // Unique
        val point2index: BiMap<SimplePoint, Int> = HashBiMap.create()
        this.forEach { (a, b, c) ->
            listOf(a, b, c).forEach { point ->
                point2index.getOrPut(point) {
                    point2index.size
                }
            }
        }
        println("vertices:${point2index.size}")
        pw.println("element vertex ${point2index.size}")
        pw.println("property float x")
        pw.println("property float y")
        pw.println("property float z")

        println("faces:${this.size}")
        pw.println("element face ${this.size}")
        pw.println("property list uchar int vertex_index")
        pw.println("end_header")

        point2index.keys.forEach { point ->
            pw.println("${point.x} ${point.y} ${point.z}")
        }

        this.forEach {
            pw.println("3 ${point2index[it.first]} ${point2index[it.second]} ${point2index[it.third]}")
        }
    }
    println("Done writing ply.")
}

/**
 * Based on the description in wikipedia
 * solid $name
 * facet normal 0 0 0
 * outer loop
 *   vertex v1x v1y v1z
 *   vertex v2x v2y v2z
 *   vertex v3x v3y v3z
 * endloop
 * endfacet
 * endsolid $name
 */
fun Collection<Facet>.saveToStl(file: File, name: String = "custom") {
    file.printWriter(StandardCharsets.UTF_8).use { pw ->
        pw.println("solid $name")
        this.forEach {
            pw.println("facet normal 0 0 1")
            pw.println(" outer loop")
            pw.println("  vertex ${it.first.x} ${it.first.y} ${it.first.z}")
            pw.println("  vertex ${it.second.x} ${it.second.y} ${it.second.z}")
            pw.println("  vertex ${it.third.x} ${it.third.y} ${it.third.z}")
            pw.println(" endloop")
            pw.println("endfacet")
        }
        pw.println("endsolid $name")
    }
    println("Done writing stl.")
}