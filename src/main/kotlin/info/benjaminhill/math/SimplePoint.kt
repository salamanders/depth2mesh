package info.benjaminhill.math

/**
 * Simple linear algebra and geometry functions designed to operate on doubles[].
 * <ul>
 * <li>points are projected via right multiplication. E.g., p' = Mp.</li>
 * <li>Roll: rotation around X. Pitch: rotation around Y. Yaw: rotation around Z.</li>
 * <li>Roll Pitch Yaw are evaluated in the order: roll, pitch, then yaw. I.e.,
 * rollPitchYawToMatrix(rpy) = rotateZ(rpy[2]) * rotateY(rpy[1]) * rotateX(rpy[0])</li>
 * </ul>
 *
 **/

typealias SimplePoint = SimpleVector

fun simplePointOf(x: Double, y: Double, z: Double): SimplePoint = doubleArrayOf(x, y, z)

fun simplePointOf(x: Int, y: Int, z: Int): SimplePoint = simplePointOf(x.toDouble(), y.toDouble(), z.toDouble())

val SimplePoint.x: Double
    get() = this[0]

val SimplePoint.y: Double
    get() = this[1]

val SimplePoint.z: Double
    get() = this[2]

val SimplePoint.dimension: Int
    get() = 3

fun SimplePoint.pretty(): String = "[x:$x, y:$y, z:$z]"