package info.benjaminhill.depth2mesh

import org.apache.commons.math3.linear.ArrayRealVector

/**
 * Helpers for passing around the points, converting, etc.
 */
typealias Point = ArrayRealVector

fun Point(vararg elts: Double): Point = Point(doubleArrayOf(*elts))

fun Point(vararg elts: Int): Point = Point(elts.map(Int::toDouble))

fun Point(elts: Collection<Double>): Point = Point(elts.toDoubleArray())

val Point.x: Double
    get() = this.getEntry(0)

val Point.y: Double
    get() = this.getEntry(1)

val Point.z: Double
    get() = this.getEntry(2)

fun Point.multiplyToSelf(other: Point): Point {
    for (i in dataRef.indices) {
        dataRef[i] *= other.dataRef[i]
    }
    return this
}

internal typealias Facet = Triple<Point, Point, Point>