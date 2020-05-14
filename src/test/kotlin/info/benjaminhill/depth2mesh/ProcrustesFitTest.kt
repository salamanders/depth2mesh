package info.benjaminhill.depth2mesh

import org.junit.Test
import org.scijava.java3d.Transform3D
import org.scijava.vecmath.Point3d
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sin
import kotlin.random.Random
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import kotlin.test.asserter


class ProcrustesFitTest {

    private val squareWithDivot2D = listOf(
        Point(-1.0, -1.0),
        Point(1.0, -1.0),
        Point(1.0, 1.0),
        Point(-1.0, 1.0),
        Point(0.0, 0.0),
    )

    private val messyCloud3D = Array(1_000) {
        Point(DoubleArray(3) { Math.random() * 500 + 200 })
    }.toList()

    @Test
    fun identical() {
        val cloud0 = PointCloud(squareWithDivot2D)
        val cloud1 = PointCloud(squareWithDivot2D)
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1))
        assertEqualsDouble(0.0, pf.err)
        assertEqualsDouble(pf.err, pf.getEuclideanError())
        assertEqualsDouble(1.0, pf.scale)
        assertEqualsArray(doubleArrayOf(0.0, 0.0), pf.translation.toArray())
    }

    @Test
    fun identicalMess() {
        val cloud0 = PointCloud(messyCloud3D)
        val cloud1 = PointCloud(messyCloud3D)
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1))
        assertEqualsDouble(0.0, pf.err, epsilon = 0.01)
        assertEqualsDouble(pf.err, pf.getEuclideanError())
        assertEqualsDouble(1.0, pf.scale)
        assertEqualsArray(doubleArrayOf(0.0, 0.0, 0.0), pf.translation.toArray())
    }


    @Test
    fun linearTranslation() {
        val translate = Point(0.2, 0.1)
        val cloud0 = PointCloud(squareWithDivot2D)
        val cloud1 = PointCloud(squareWithDivot2D.map {
            it.add(translate)
        })
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1))
        assertEqualsDouble(0.0, pf.err)
        assertEqualsDouble(pf.err, pf.getEuclideanError())
        assertEqualsDouble(1.0, pf.scale)
        assertEqualsArray(translate.toArray(), pf.translation.toArray())
    }

    @Test
    fun linearTranslationMess() {
        val translate = Point(0.02 , -0.01, 0.01)
        val cloud0 = PointCloud(messyCloud3D)
        val cloud1 = PointCloud(messyCloud3D.map {
            it.add(translate)
        })
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1))
        assertEqualsDouble(0.0, pf.err, epsilon = 0.01)
        assertEqualsDouble(pf.err, pf.getEuclideanError())
        assertEqualsDouble(1.0, pf.scale)
        assertEqualsArray(translate.toArray(), pf.translation.toArray())
    }

    @Test
    fun shrink() {
        val scaleX = 0.9
        val cloud0 = PointCloud(squareWithDivot2D)
        val cloud1 = PointCloud(squareWithDivot2D.map {
            doubleArrayOf(it.x * scaleX, it.y) // non uniform scale
        })
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1), allowScaling=true)
        assertTrue(pf.err < 0.2)
        assertTrue(pf.err > 0.0)
        assertEqualsDouble(0.95, pf.scale) // half of scale
        assertEqualsArray(doubleArrayOf(0.0, 0.0), pf.translation.toArray())
    }

    @Test
    fun grow() {
        val scale = 1.001
        val cloud0 = PointCloud(messyCloud3D)
        val cloud1 = PointCloud(messyCloud3D.map {
            it.mapMultiply(scale) as Point // uniform
        })
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1), allowScaling=true)
        println(pf)
        assertTrue(pf.err < 0.2)
        assertTrue(pf.err >= 0.0)
        assertEqualsDouble(scale, pf.scale)
        assertEqualsArray(doubleArrayOf(0.0, 0.0, 0.0), pf.translation.toArray())
    }


    @Test
    fun smallRotation() {
        val rotR: Radian = (5.0).toRadian()
        val cloud0 = PointCloud(squareWithDivot2D)
        val cloud1 = PointCloud(squareWithDivot2D.map {
            doubleArrayOf(it.x * cos(rotR) - it.y * sin(rotR), it.x * sin(rotR) + it.y * cos(rotR))
        })
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1))
        println(pf)
        assertEqualsDouble(0.0, pf.err)
        assertEqualsDouble(1.0, pf.scale)
        assertEqualsArray(doubleArrayOf(0.0, 0.0), pf.translation.toArray())
        assertEqualsDouble(rotR, acos(pf.orthogonalRotation.getEntry(0, 0)))

        // TODO: Check rotation matrix
    }

    @Test
    fun jittery() {
        val cloud0 = PointCloud(messyCloud3D)
        val cloud1 = PointCloud(messyCloud3D.map {
            Point(
                it.x + Random.nextDouble(-5.0, 5.0),
                it.y + Random.nextDouble(-5.0, 5.0),
                it.z + Random.nextDouble(-5.0, 5.0),
            )
        })
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1))
        println(pf)
        assertTrue(pf.err > 100)
        assertEqualsDouble(1.0, pf.scale)
        // Zero rotation
        assertEqualsDouble(0.0, pf.orthogonalRotation.getEntry(0,1), epsilon = 0.05)
        assertEqualsArray(doubleArrayOf(0.0, 0.0, 0.0), pf.translation.toArray(), epsilon = 2.0)
    }

    @Test
    fun smallRotationMessy() {
        val rot: Radian = (-0.1).toRadian()
        val transBoth = Transform3D().apply {
            mul(Transform3D() .apply {
                rotY(rot)
            })
            mul(Transform3D() .apply {
                rotZ(rot)
            })
        }
        val cloud0 = PointCloud(messyCloud3D)
        val cloud1 = PointCloud(messyCloud3D.map {
            val transformedPoint = Point3d(doubleArrayOf(it.x, it.y, it.z))
            transBoth.transform(transformedPoint)
            doubleArrayOf(transformedPoint.x, transformedPoint.y, transformedPoint.z)
        })
        val pf = ProcrustesFit(cloud0.getNearestNeighbors(cloud1))
        println(pf)
        assertEqualsDouble(0.0, pf.err)
        assertEqualsDouble(1.0, pf.scale)
        // Check rotation
        assertEqualsDouble(0.0017, pf.orthogonalRotation.getEntry(0,1))
        assertEqualsArray(doubleArrayOf(0.0, 0.0, 0.0), pf.translation.toArray())
    }
}


internal fun assertEqualsDouble(
    expected: Double,
    actual: Double?,
    epsilon: Double = 0.001,
    message: String? = null
) {
    asserter.assertNotNull(null, actual)
    asserter.assertTrue(
        { (if (message == null) "" else "$message. ") + "Expected <$expected>, actual <$actual>, should differ no more than <$epsilon>." },
        abs(expected - actual!!) <= epsilon
    )
}

internal fun assertEqualsArray(
    expected: DoubleArray,
    actual: DoubleArray?,
    epsilon: Double = 0.001,
    message: String? = null
) {
    asserter.assertNotNull(null, actual)
    assertEquals(expected.size, actual?.size)
    for (i in expected.indices) {
        assertEqualsDouble(expected[i], actual?.get(i), epsilon=epsilon, message = "${message?:""} Index $i")
    }
}

internal typealias Degree = Double
internal typealias Radian = Double

internal fun Degree.toRadian(): Radian = (this / 180) * Math.PI