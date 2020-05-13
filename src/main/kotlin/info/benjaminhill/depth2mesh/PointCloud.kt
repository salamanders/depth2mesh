package info.benjaminhill.depth2mesh

import ch.ethz.globis.phtree.PhTreeF
import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector

/**
 * A collection of some number of vectors, all with the same dimensionality.
 * Please don't go adding more points later.
 */
class PointCloud(vecs: Collection<RealVector>) : ArrayList<RealVector>(vecs) {
    constructor(vararg vecs: RealVector) : this(vecs.asList())

    private val dimension: Int

    init {
        require(isNotEmpty()) { "No empty PointClouds" }
        dimension = first().dimension
        for (vec in this) {
            require(vec.dimension == dimension) { "All vector dimensions must be the same within a point cloud." }
        }
    }

    /**
     * Center of the cloud
     */
    fun getMeanVec(): RealVector {
        val total = fold(ArrayRealVector(dimension)) { acc, realVector ->
            return acc.add(realVector)
        }
        return total.mapDivide(size.toDouble())
    }

    /**
     * Batch transform to a PhTreeF.
     * TBD if there are faster ways to batch-transform the points, or ways to batch-add to the tree.
     */
    fun toTree(): PhTreeF<DoubleArray> = PhTreeF.create<DoubleArray>(dimension).also { tree ->
        map { it.toArray() }.forEach { tree.put(it, it) }
    }

    /**
     * Export the data for reuse.
     */
    fun toDataMatrix(meanX: RealVector? = null): RealMatrix = MatrixUtils.createRealMatrix(dimension, size).also { m ->
        val mean = MatrixUtils.createRealVector(meanX?.toArray())
        forEachIndexed { index, realVector ->
            var cv = MatrixUtils.createRealVector(realVector.toArray())
            mean?.let {
                cv = cv.subtract(it)
            }
            m.setColumnVector(index, cv)
        }
    }
}



