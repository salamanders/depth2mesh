# depth2mesh

Align 3d depth scans (like from a Pixel 4 Camera) 
into a single higher quality mesh.

From the Java code in [imagingbook ProcrustesFit](https://github.com/imagingbook/imagingbook-common/blob/master/src/main/java/imagingbook/pub/geometry/fitting/ProcrustesFit.java)


## TODO

Is it working?

Better to use http://www.open3d.org/docs/release/tutorial/Advanced/multiway_registration.html


## Reading

* [Kabsch Algorithm](https://en.wikipedia.org/wiki/Kabsch_algorithm)
* https://github.com/Fidentis/Analyst/blob/master/Comparison/src/cz/fidentis/comparison/procrustes/ProcrustesAnalysis.java
* https://github.com/charnley/rmsd/blob/master/rmsd/calculate_rmsd.py
* [Finding Optimal Rotation And Translation Between Corresponding 3d Points](http://nghiaho.com/?page_id=671)
* [Apache Commons SingularValueDecomposition](https://commons.apache.org/proper/commons-math/javadocs/api-3.0/org/apache/commons/math3/linear/SingularValueDecomposition.html)
* [Apache Commons Rotation](https://commons.apache.org/proper/commons-math/javadocs/api-3.0/org/apache/commons/math3/linear/SingularValueDecomposition.htmlhttp://commons.apache.org/proper/commons-math/javadocs/api-2.2/org/apache/commons/math/geometry/Rotation.html#Rotation(org.apache.commons.math.geometry.Vector3D,%20org.apache.commons.math.geometry.Vector3D))
* [Wahba's problem](https://en.wikipedia.org/wiki/Wahba%27s_problem)
* [Averages of Rotations and Orientations in 3-space](http://www.cs.unc.edu/techreports/01-029.pdf)
