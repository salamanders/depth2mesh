package info.benjaminhill.depth2mesh

import com.google.common.collect.BiMap
import com.google.common.collect.HashBiMap
import org.scijava.vecmath.Point3d
import java.awt.Color
import java.io.File
import java.nio.charset.StandardCharsets
import javax.imageio.ImageIO

fun main() {
    val face = ImageIO.read(File("DATA/faces/image_1588368657814.png"))

    val zDist = face.mapEach { x, y ->
        val rgb = Color(face.getRGB(x, y))
        (rgb.red * 256) + rgb.green
    }

    val confidence = face.mapEach { x, y ->
        val rgb = Color(face.getRGB(x, y))
        rgb.blue / 256.0
    }

    // Arm length
    val zThreshold = 1_000
    println("zThreshold:$zThreshold")

    val mesh = mutableListOf<Facet>()
    var countFarZ = 0
    var countBadTri = 0
    var countBadConf = 0
    var countAll = 0

    for (x in 0 until face.width - 1) {
        pixel@ for (y in 0 until face.height - 1) {
            countAll++
            val quad = mutableListOf<Point3d>()

            for (py in y..y + 1) {
                for (px in x..x + 1) {
                    if (confidence[px][py] < .3) {
                        countBadConf++
                        continue@pixel
                    }

                    if (zDist[px][py] > zThreshold) {
                        countFarZ++
                        continue@pixel
                    }
                    quad.add(Point3d(px.toDouble(), py.toDouble(), (-zDist[px][py]).toDouble()))
                }
            }
            // Quad will always be 4 long
            if (quad.map { it.z }.relativeStandardDeviation() > 0.5) {
                countBadTri++
                continue@pixel
            }

            mesh.add(Facet(quad[0], quad[1], quad[2]))
            mesh.add(Facet(quad[0], quad[2], quad[3]))
        }
    }

    println(
            "${countBadConf.toDouble() / countAll} bad conf, " +
                    "${countFarZ.toDouble() / countAll} far z, " +
                    "${countBadTri.toDouble() / countAll} bad tri," +
                    "$countAll written."
    )

    mesh.saveToPly(File("image01.ply"))
    mesh.saveToStl(File("image01.stl"))
}

fun Collection<Facet>.saveToPly(file: File) {
    file.printWriter(StandardCharsets.US_ASCII).use { pw ->
        pw.println("ply")
        pw.println("format ascii 1.0")
        pw.println("comment by Benjamin Hill")
        pw.println("comment Depth Map from smartphone")

        // Unique
        val point2index: BiMap<Point3d, Int> = HashBiMap.create()
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
            pw.println("  vertex ${it.first}")
            pw.println("  vertex ${it.second}")
            pw.println("  vertex ${it.third}")
            pw.println(" endloop")
            pw.println("endfacet")
        }
        pw.println("endsolid $name")
    }
    println("Done writing stl.")
}
