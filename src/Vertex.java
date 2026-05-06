import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.jme3.math.Vector3f;

/** A vertex in the triangle mesh, storing its position, adjacency, and parameterization state. */
public class Vertex {
    Vector3f pos;
    List<Vertex> neighbors = new ArrayList<>();

    // f: geodesic distance from seed (row coordinate).
    // g: angular coordinate computed by BFGS optimization.
    double f = Double.MAX_VALUE;
    double g = 1d;

    Vector3f gradF = new Vector3f();
    Vector3f gradG = new Vector3f();
    Vector3f rotatedGradF = new Vector3f();

    boolean finalized = false;  // used by Dijkstra to mark settled vertices
    boolean isBoundary = false; // boundary vertices have g fixed to 0

    // Used by findLongestGeodesic to reconstruct the seam path.
    Vertex predecessor;
    double distance = Double.MAX_VALUE;

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        return pos.equals(((Vertex) o).pos);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(pos);
    }
}
