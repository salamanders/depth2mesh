package info.benjaminhill.depth2mesh

import com.google.common.collect.BiMap
import com.google.common.collect.HashBiMap
import java.awt.Color
import java.awt.image.BufferedImage
import java.io.File
import java.nio.charset.StandardCharsets
import javax.imageio.ImageIO


fun getAllClouds(
    maxDistance: Int = 300,
    minConfidence: Double = 0.8,
    zStretch: Double = 1.8,
    decimate: Double = 2.0
) = File("DATA/faces/")
    .walk()
    .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
    .sortedBy { it.nameWithoutExtension }
    .map {
        ImageIO.read(it).toPointCloud(maxDistance, minConfidence)
    }
    .onEach { it.multiplyAllToSelf(Point(1.0, 1.0, zStretch)) } // Streeetch Z
    .map { it.decimate(decimate) }
    .toList().also {
        println("Loaded images into ${it.size} PointClouds, and decimated.")
    }

/**
 * Hacky depth map in a PNG - distance=red*256+green, confidence=blue
 */
internal fun BufferedImage.getDistanceConfidence(x: Int, y: Int): Pair<Int, Double> {
    val rgb = Color(getRGB(x, y))
    val dist = (rgb.red * 256) + rgb.green
    val confidence = rgb.blue / 256.0
    return Pair(dist, confidence)
}

internal fun BufferedImage.toPointCloud(maxDistance: Int = Int.MAX_VALUE, minConfidence: Double = 0.0) =
    PointCloud(mapEach { x, y ->
        val (distance, confidence) = this.getDistanceConfidence(x, y)
        if (distance <= maxDistance && confidence >= minConfidence) {
            Point(x, y, distance)
        } else {
            null
        }
    }.flatten().filterNotNull())


fun Collection<Facet>.saveToPly(file: File) {
    file.printWriter(StandardCharsets.US_ASCII).use { pw ->
        pw.println("ply")
        pw.println("format ascii 1.0")
        pw.println("comment by Benjamin Hill")
        pw.println("comment Depth Map from smartphone")

        // Unique
        val point2index: BiMap<Point, Int> = HashBiMap.create()
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

/**
 * Hopefully import into
 * http://fabacademy.org/archives/2014/tutorials/pointcloudToSTL.html
 */
fun pointsToAsc(nameWithoutExtension: String, pc: Collection<Point>) {
    val file = File("$nameWithoutExtension.asc")
    file.printWriter().use { pw ->
        pc.forEach { vector ->
            pw.println(vector.dataRef.joinToString(" "))
        }
    }
}

/** Helper to "do stuff" to each pixel */
inline fun <reified T> BufferedImage.mapEach(fn: (x: Int, y: Int) -> T) = Array(this.width) { x ->
    Array(this.height) { y ->
        fn(x, y)
    }
}