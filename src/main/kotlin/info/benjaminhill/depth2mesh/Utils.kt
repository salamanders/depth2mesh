package info.benjaminhill.depth2mesh

import org.nield.kotlinstatistics.standardDeviation
import org.scijava.vecmath.Point3d
import java.awt.image.BufferedImage

inline fun <reified T> BufferedImage.mapEach(fn: (x: Int, y: Int) -> T) = Array(this.width) { x ->
    Array(this.height) { y ->
        fn(x, y)
    }
}

inline fun BufferedImage.forEach(fn: (x: Int, y: Int) -> Unit) = Array(this.width) { x ->
    Array(this.height) { y ->
        fn(x, y)
    }
}

fun Collection<Double>.relativeStandardDeviation() = this.standardDeviation() / this.average()

fun LongArray.distSq(other: LongArray): Long {
    var dist = 0L
    for (i in this.indices) {
        dist += (this[i] - other[i]) * (this[i] - other[i])
    }
    return dist
}

typealias Facet = Triple<Point3d, Point3d, Point3d>