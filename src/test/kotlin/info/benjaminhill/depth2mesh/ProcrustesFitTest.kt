package info.benjaminhill.depth2mesh

import org.junit.Test
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.asserter


class ProcrustesFitTest {

    private val squareWithDivot = listOf(
        doubleArrayOf(-1.0, -1.0),
        doubleArrayOf(1.0, -1.0),
        doubleArrayOf(1.0, 1.0),
        doubleArrayOf(-1.0, 1.0),
        doubleArrayOf(0.0, 0.0),
    )

    @Test
    fun identical() {
        val pf = arraysToProcrustes(squareWithDivot, squareWithDivot)
        assertEqualsDouble(0.0, pf.err)
        assertEqualsDouble(pf.err, pf.getEuclideanError())
        assertEqualsArray(doubleArrayOf(0.0, 0.0), pf.translation.toArray())
    }


    @Test
    fun linearTranslation() {
        val xd = 1.1
        val yd = 0.5
        val pf = arraysToProcrustes(squareWithDivot, squareWithDivot.map {
            doubleArrayOf(it[0] + xd, it[1] + yd)
        })
        println(pf)
        assertEquals(0.0, pf.err)
        assertEquals(pf.err, pf.getEuclideanError())
        assertEquals(0.0, pf.scale)
        assertEqualsArray(doubleArrayOf(xd, yd), pf.translation.toArray())
    }

/*

    // Wobbly, shifted +1x, slight counter-clockwise rotation
    private val pc1 = PointCloud(listOf(
        doubleArrayOf(1.1, 0.0),
        doubleArrayOf(2.1, 0.1),
        doubleArrayOf(2.1, 1.0),
        doubleArrayOf(0.9, 0.9),
        doubleArrayOf(1.5, 0.5),
    ).map { ArrayRealVector(it) })

    private  val tree0: PhTreeF<DoubleArray> = pc0.toTree()
    private  val tree1: PhTreeF<DoubleArray> = pc1.toTree()
    private  val pf: ProcrustesFit = ProcrustesFit(tree0.queryExtent()
        .asSequence()
        .map { pt0 -> pt0 to tree1.nearestNeighbour(1, *pt0).next()!! }
        .toMap())
*/


}


internal fun assertEqualsDouble(expected: Double, actual: Double?, epsilon: Double = 0.0001, message: String? = null) {
    asserter.assertNotNull(null, actual)
    asserter.assertTrue(
        { (if (message == null) "" else "$message. ") + "Expected <$expected>, actual <$actual>, should differ no more than <$epsilon>." },
        abs(expected - actual!!) <= epsilon
    )
}

internal fun assertEqualsArray(
    expected: DoubleArray,
    actual: DoubleArray?,
    epsilon: Double = 0.0001,
    message: String? = null
) {
    asserter.assertNotNull(null, actual)
    assertEquals(expected.size, actual?.size)
    for (i in expected.indices) {
        assertEqualsDouble(expected[i], actual?.get(i), message = "Index $i")
    }
}