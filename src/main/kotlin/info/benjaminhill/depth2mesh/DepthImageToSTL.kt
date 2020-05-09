package info.benjaminhill.depth2mesh

import org.scijava.vecmath.Point3d
import java.awt.Color
import java.io.File
import javax.imageio.ImageIO


fun main() {
    val face = ImageIO.read(File("./DATA/faces/faces/image_1588368657814.png"))

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

    //mesh.saveToPly(File("image01.ply"))
    //mesh.saveToStl(File("image01.stl"))
}

