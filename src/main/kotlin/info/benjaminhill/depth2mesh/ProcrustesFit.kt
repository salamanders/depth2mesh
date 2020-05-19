package info.benjaminhill.depth2mesh

import org.apache.commons.math3.linear.*
import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.acos
import kotlin.math.sqrt

/**
 * Find the best rotation/translation/scale to change cloudP into cloudQ
 *
 * The dimensionality of the point cloud is determined by the RealVectors in it.
 * (They would mostly be 2D or 3D.)
 * From https://github.com/imagingbook/imagingbook-common/blob/master/src/main/java/imagingbook/pub/geometry/fitting/ProcrustesFit.java
 *
 * Always allows translation and rotation
 * Flag controls scaling (so you can first try without scaling)
 *
 * @param cloudP list of points that correlate 1:1 with Q, all with the same dimensionality
 * @param cloudQ list of points that correlate 1:1 with P, all with the same dimensionality
 */
class ProcrustesFit(
    private val cloudP: PointCloud,
    private val cloudQ: PointCloud,
    allowScaling: Boolean = false
) {
    private val dimension: Int
    val orthogonalRotation: RealMatrix
    val translation: RealVector
    val scale: Double
    val transformation: RealMatrix
    val err: Double // fitting error

    constructor(
        pointMap: Map<Point, Point>,
        allowScaling: Boolean = false
    ) : this(
        PointCloud(pointMap.keys),
        PointCloud(pointMap.values),
        allowScaling
    )

    init {
        require(cloudP.size >= 4) { "Requires at least 4 points (found ${cloudP.size})." }
        require(cloudP.size == cloudQ.size) { "Both point clouds must be the same size." }

        dimension = cloudP.first().dimension
        require(dimension == cloudQ.first().dimension) { "Point cloud's points must have the same dimensionality " }

        // always allow translation
        val meanP = cloudP.getMeanVec()
        val meanY = cloudQ.getMeanVec()

        val vP = cloudP.toDataMatrix(meanP)
        val vQ = cloudQ.toDataMatrix(meanY)

        MatrixUtils.checkAdditionCompatible(vP, vQ) // P, Q of same dimensions
        val qPt = vQ.multiply(vP.transpose()) // TODO: What is qPt?
        val svd = SingularValueDecomposition(qPt)

        // Make sure you don't have strange reflections
        val d: Double =
            if (svd.rank >= 3) qPt.det() else svd.u.det() * svd.v.det() // TODO: Is rank 3 correct?  Or should it be based on the dimensions?
        val identityMatrix = MatrixUtils.createRealIdentityMatrix(dimension)
        if (d < 0) {
            identityMatrix.setEntry(1, 1, -1.0) // TODO: this is from 2D, same values in 3D?
        }

        val normP = vP.frobeniusNorm // Uh... No idea!
        val normQ = vQ.frobeniusNorm

        orthogonalRotation = svd.u.multiply(identityMatrix).multiply(svd.v.transpose())
        scale = if (allowScaling) svd.s.multiply(identityMatrix).trace / normP.sqr() else 1.0

        val ma = MatrixUtils.createRealVector(meanP.dataRef)
        val mb = MatrixUtils.createRealVector(meanY.dataRef)
        translation = mb.subtract(orthogonalRotation.scalarMultiply(scale).operate(ma))

        // make the transformation matrix A
        val cR = orthogonalRotation.scalarMultiply(scale)
        transformation = MatrixUtils.createRealMatrix(dimension, 3) // Why 3?  Same 3 as before?
        transformation.setSubMatrix(cR.data, 0, 0)
        transformation.setColumnVector(2, translation) // Why 2?

        err = (sqrt(normQ.sqr() - (svd.s.multiply(identityMatrix).trace / normP).sqr())).let {
            if (it.isFinite()) it else 0.0
        }
    }

    /** Slow version to compare with err */
    fun getEuclideanError(): Double {
        val sR: RealMatrix = orthogonalRotation.scalarMultiply(scale)
        var errSum = 0.0
        for (i in cloudP.indices) {
            val p: RealVector = cloudP[i]
            val q: RealVector = cloudQ[i]
            val pTransformed = sR.operate(p).add(translation)
            val e = pTransformed.subtract(q).norm
            errSum += e.sqr()
        }
        return sqrt(errSum)
    }


    /** Get the cloud P, transformed as close as possible to Q */
    fun getPTransformed(): PointCloud {
        val sR: RealMatrix = orthogonalRotation.scalarMultiply(scale)
        return PointCloud(cloudP.map { p ->
            sR.operate(p).add(translation) as Point
        })
    }

    override fun toString(): String = StringBuilder().apply {
        append("estimated alpha: ${acos(orthogonalRotation.getEntry(0, 0)).rnd()}\n") // What is this?
        append("estimated rotation:${orthogonalRotation.data.prettyPrint()}\n")
        append("estimated translation: $translation\n")
        append("estimated scale: ${scale.rnd()}\n")
        append("transformation: ${transformation.data.prettyPrint()}\n")
        append("RMS fitting error: ${err.rnd()}, euclidean fitting error: ${getEuclideanError().rnd()}")
    }.toString()

    companion object {
        fun RealMatrix.det() = LUDecomposition(this).determinant
        fun Double.sqr() = this * this
        fun Double.rnd(digits: Int = 4): Double = when {
            this.isNaN() -> Double.NaN
            this.isInfinite() -> Double.POSITIVE_INFINITY
            else -> BigDecimal(this).setScale(digits, RoundingMode.HALF_EVEN).toDouble()
        }

        fun Array<DoubleArray>.prettyPrint() = "\n  " + joinToString("\n  ") { row ->
            row.joinToString(" ") { it.rnd().toString() }
        }
    }
}