package info.benjaminhill.depth2mesh

import ch.ethz.globis.phtree.PhTree
import org.scijava.java3d.Transform3D
import org.scijava.vecmath.Point3d
import java.awt.Color
import java.io.File
import javax.imageio.ImageIO

typealias PointCloud = Array<Point3d>

/**
 * Copies the data from a (hopefully fast) Array<Point3d> to a PhTree.
 * TBD if there are faster ways to batch-transform the points, or ways to batch-add to the tree.
 */
private fun pointCloudToTransformedTree(pc: PointCloud, transformer: Transform3D = Transform3D()): PhTree<LongArray> {
    val tree: PhTree<LongArray> = PhTree.create(3)
    val transformedPoint = Point3d()
    pc.map { p3d ->
        transformer.transform(p3d, transformedPoint)
        val long3d = longArrayOf(transformedPoint.x.toLong(), transformedPoint.y.toLong(), transformedPoint.z.toLong())
        tree.put(long3d, long3d)
    }
    return tree
}

/**
 * Hacky depth map in a PNG - distance=red*256+green, confidence=blue
 */
private fun imageFileToPointCloud(imageFile: File): PointCloud {
    println(imageFile.name)
    val faceImage = ImageIO.read(imageFile)
    return faceImage.mapEach { x, y ->
        val rgb = Color(faceImage.getRGB(x, y))
        val dist = (rgb.red * 256) + rgb.green
        val confidence = rgb.blue / 256.0
        if (dist < 1_000 && confidence > 0.5) {
            Point3d(x.toDouble(), y.toDouble(), dist.toDouble())
        } else {
            null
        }
    }.flatten().filterNotNull().toTypedArray()
}


fun main() {
    // Load up first 2 images as a set of long points in 3d.
    val initialPointClouds = File("../DATA/faces/").walk().filter {
        it.isFile &&
                it.canRead() &&
                setOf("jpg", "jpeg", "png").contains(it.extension.toLowerCase())
    }.take(2).map { imageFileToPointCloud(it) }.toMutableList()

    val (baselineCloud, secondCloud) = initialPointClouds
    val baselineTree = pointCloudToTransformedTree(baselineCloud)

    val fractionOfDegree = 0.2
    val everyNth = 100
    val rotations: List<Pair<Triple<Double, Double, Double>, Double>> =
            (-1..1).map { it * fractionOfDegree }.map { xd ->
                (-1..1).map { it * fractionOfDegree }.map { yd ->
                    (-1..1).map { it * fractionOfDegree }.map { zd ->
                        val xform = Transform3D().apply {
                            val xformRotX = Transform3D()
                            val xformRotY = Transform3D()
                            val xformRotZ = Transform3D()

                            xformRotX.rotX(xd)
                            xformRotY.rotY(yd)
                            xformRotZ.rotZ(zd)

                            this.mul(xformRotX)
                            this.mul(xformRotY)
                            this.mul(xformRotZ)
                        }


                        var secondDistRot = 0.0
                        val secondTreeRot = pointCloudToTransformedTree(secondCloud, xform)
                        baselineTree.queryExtent()
                                .asSequence()
                                .chunked(everyNth)
                                .map { it.first() }
                                .forEach { coordinates ->
                                    secondDistRot += coordinates.distSq(secondTreeRot.nearestNeighbour(1, *coordinates).next()!!)
                                }
                        println("rot: $xd,$yd,$zd  = $secondDistRot")
                        Pair(Triple(xd, yd, zd), secondDistRot)
                    }
                }.flatten()
            }.flatten().sortedBy { it.second }

    val (bestTransform, bestDistance) = rotations.first()
    println("Best Rotation: ${bestTransform.first},${bestTransform.second},${bestTransform.third}  dist:$bestDistance")
}