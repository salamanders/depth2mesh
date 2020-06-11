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

typealias SimplePoint3d = SimpleVector

fun simplePoint3dOf(x: Double, y: Double, z: Double): SimplePoint3d = doubleArrayOf(x, y, z)

fun simplePoint3dOf(x: Int, y: Int, z: Int): SimplePoint3d = simplePoint3dOf(x.toDouble(), y.toDouble(), z.toDouble())

val SimplePoint3d.x: Double
    get() = this[0]

val SimplePoint3d.y: Double
    get() = this[1]

val SimplePoint3d.z: Double
    get() = this[2]

val SimplePoint3d.dimension: Int
    get() = 3

fun SimplePoint3d.pretty(): String = "[x:$x, y:$y, z:$z]"