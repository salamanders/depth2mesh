package info.benjaminhill.depth2mesh

import org.junit.Test

import org.junit.Assert.*
import java.io.File
import javax.imageio.ImageIO

class DepthIOTest {

    @Test
    fun testFaceFile() {
        val faceFile = File("DATA/faces/")
            .walk()
            .filter { it.isFile && it.canRead() && "png" == it.extension.toLowerCase() }
            .sortedBy { it.nameWithoutExtension }
            .first()

        val image = ImageIO.read(faceFile)
        val imageWidth = image.width
        val imageHeight = image.height
        val mesh = mutableListOf<Facet>()

        for (y in 0 until imageHeight - 1) {
            for (x in 0 until imageWidth - 1) {
                // x0y0, x0y1, x1y0, x1y1
                val distances: List<Point> = (0..1).map { dy ->
                    (0..1).map { dx ->
                        val (dist, conf) = image.getDistanceConfidence(x + dx, y + dy)
                        if (dist < 500 && conf > 0.5) {
                            Point(x + dx, y + dy, dist * 2)
                        } else {
                            null
                        }
                    }
                }.flatten().filterNotNull()

                if (distances.size == 4) {
                    mesh.add(Facet(distances[0], distances[3], distances[1]))
                    mesh.add(Facet(distances[0], distances[2], distances[3]))
                }
            }
        }
        mesh.saveToPly(File("image01.ply"))
        mesh.saveToStl(File("image01.stl"))
    }
}