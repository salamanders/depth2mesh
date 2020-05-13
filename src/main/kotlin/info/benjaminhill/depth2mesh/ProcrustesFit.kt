package info.benjaminhill.depth2mesh

import org.apache.commons.math3.linear.*
import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.acos
import kotlin.math.sqrt

/**
 * Find the best rotation/translation/scale between two point clouds.
 * The dimensionality of the point cloud is determined by the RealVectors in it.  They would mostly be 2D or 3D.
 * From https://github.com/imagingbook/imagingbook-common/blob/master/src/main/java/imagingbook/pub/geometry/fitting/ProcrustesFit.java
 *
 * Always allows translation, scaling, and rotation
 *
 * @param cloudP list of points that correlate 1:1 with Q, all with the same dimensionality
 * @param cloudQ list of points that correlate 1:1 with P, all with the same dimensionality
 */
class ProcrustesFit(
    private val cloudP: PointCloud,
    private val cloudQ: PointCloud
) {
    private val dimension: Int
    val orthogonalRotation: RealMatrix
    val translation: RealVector
    val scale: Double
    val transformation: RealMatrix
    val err: Double // fitting error

    constructor(pairs: Map<DoubleArray, DoubleArray>) : this(
        pairs.map { ArrayRealVector(it.key) },
        pairs.map { ArrayRealVector(it.value) })

    constructor(pairs: List<Pair<RealVector, RealVector>>) : this(pairs.map { it.first }, pairs.map { it.second })
    constructor(pointsP: List<RealVector>, pointsQ: List<RealVector>) : this(PointCloud(pointsP), PointCloud(pointsQ))


    init {
        require(cloudP.size >= 4) { "Requires at least 4 points (found ${cloudP.size})." }
        require(cloudP.size == cloudQ.size) { "Both point clouds must be the same size." }

        dimension = cloudP.first().dimension
        for (v in cloudP) {
            require(v.dimension == dimension) { "Found a point in cloudP with the wrong dimension." }
        }
        for (v in cloudQ) {
            require(v.dimension == dimension) { "Found a point in cloudQ with the wrong dimension." }
        }

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
        scale = svd.s.multiply(identityMatrix).trace / normP.sqr()

        val ma = MatrixUtils.createRealVector(meanP.toArray())
        val mb = MatrixUtils.createRealVector(meanY.toArray())
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
            val p: RealVector = ArrayRealVector(cloudP[i].toArray())
            val q: RealVector = ArrayRealVector(cloudQ[i].toArray())
            val pp = sR.operate(p).add(translation)
            val e = pp.subtract(q).norm
            errSum += e.sqr()
        }
        return sqrt(errSum)
    }

    override fun toString(): String = StringBuilder().apply {
        append("estimated alpha: ${acos(orthogonalRotation.getEntry(0, 0)).rnd()}\n") // What is this?
        append("estimated rotation:${orthogonalRotation.data.prettyPrint()}\n")
        append("estimated translation: $translation\n")
        append("estimated scale: ${scale.rnd()}\n")
        append("transformation: ${transformation.data.prettyPrint()}\n")
        append("RMS fitting error: $err, euclidean fitting error: ${getEuclideanError().rnd()}")
    }.toString()

    companion object {
        fun RealMatrix.det() = LUDecomposition(this).determinant
        fun Double.sqr() = this * this
        fun Double.rnd(digits: Int = 4) = BigDecimal(this).setScale(digits, RoundingMode.HALF_EVEN).toDouble()
        fun Array<DoubleArray>.prettyPrint() = "\n  " + joinToString("\n  ") { row ->
            row.joinToString(" ") { it.rnd().toString() }
        }
    }
}