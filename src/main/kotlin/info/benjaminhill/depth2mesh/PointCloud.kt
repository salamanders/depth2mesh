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
class PointCloud(allVectors: Collection<Point>) : ArrayList<Point>(allVectors) {

    constructor(allDoubleArrays: List<DoubleArray>) : this(allDoubleArrays.map { Point(*it) })

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
    private fun toTree(): PhTreeF<DoubleArray> = PhTreeF.create<DoubleArray>(dimension).also { tree ->
        map { it.toArray() }.forEach { tree.put(it, it) }
    }

    /**
     * Map of each point in this cloud to nearest neighbor in the other cloud
     */
    fun getNearestNeighbors(other: PointCloud): Map<Point, Point> {
        val otherTree = other.toTree()
        return this.toTree().queryExtent().asSequence()
            .map { pt0 -> pt0 to otherTree.nearestNeighbour(1, *pt0).next()!! }
            .map { Point(it.first) to Point(it.second) }
            .toMap()
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



