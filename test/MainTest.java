import com.jme3.math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;
import java.util.TreeMap;

import static org.junit.jupiter.api.Assertions.*;

class MainTest {

    private static Vertex vertexWithF(double f) {
        Vertex v = new Vertex();
        v.pos = new Vector3f((float) f, 0f, 0f);
        v.f = f;
        return v;
    }

    private static Vertex vertexAt(float x, float y, float z) {
        Vertex v = new Vertex();
        v.pos = new Vector3f(x, y, z);
        return v;
    }

    // --- areClose ---

    @Test
    void areClose_withinThreshold_returnsTrue() {
        assertTrue(Main.areClose(1.0, 1.0 + Main.THRESHOLD * 0.5));
    }

    @Test
    void areClose_beyondThreshold_returnsFalse() {
        assertFalse(Main.areClose(1.0, 1.0 + Main.THRESHOLD * 2));
    }

    @Test
    void areClose_equalValues_returnsTrue() {
        assertTrue(Main.areClose(0.5, 0.5));
    }

    // --- groupByRows ---

    @Test
    void groupByRows_closeVerticesMergedIntoSameRow() {
        Vertex v1 = vertexWithF(1.0);
        Vertex v2 = vertexWithF(1.0 + Main.THRESHOLD * 0.5);
        Vertex v3 = vertexWithF(2.0);

        TreeMap<Double, List<Vertex>> rows = Main.groupByRows(Arrays.asList(v1, v2, v3));

        assertEquals(2, rows.size());
        assertEquals(2, rows.firstEntry().getValue().size());
    }

    @Test
    void groupByRows_distantVerticesInSeparateRows() {
        Vertex v1 = vertexWithF(0.0);
        Vertex v2 = vertexWithF(1.0);
        Vertex v3 = vertexWithF(2.0);

        TreeMap<Double, List<Vertex>> rows = Main.groupByRows(Arrays.asList(v1, v2, v3));

        assertEquals(3, rows.size());
    }

    @Test
    void groupByRows_emptyInput_returnsEmptyMap() {
        assertTrue(Main.groupByRows(Arrays.asList()).isEmpty());
    }

    @Test
    void groupByRows_singleVertex_oneRow() {
        Vertex v = vertexWithF(0.5);
        TreeMap<Double, List<Vertex>> rows = Main.groupByRows(Arrays.asList(v));
        assertEquals(1, rows.size());
        assertEquals(1, rows.firstEntry().getValue().size());
    }

    @Test
    void groupByRows_rowsOrderedByFValue() {
        Vertex v1 = vertexWithF(3.0);
        Vertex v2 = vertexWithF(1.0);
        Vertex v3 = vertexWithF(2.0);

        TreeMap<Double, List<Vertex>> rows = Main.groupByRows(Arrays.asList(v1, v2, v3));

        assertEquals(3, rows.size());
        assertEquals(1.0, rows.firstKey(), 1e-9);
        assertEquals(3.0, rows.lastKey(), 1e-9);
    }

    // --- findClosestVertex ---

    @Test
    void findClosestVertex_returnsNearestByPosition() {
        Vertex v1 = vertexAt(0f, 0f, 0f);
        Vertex v2 = vertexAt(1f, 0f, 0f);
        Vertex v3 = vertexAt(5f, 0f, 0f);

        Vertex result = Main.findClosestVertex(new Vector3f(0.9f, 0f, 0f), Arrays.asList(v1, v2, v3));

        assertEquals(v2, result);
    }

    @Test
    void findClosestVertex_emptyList_returnsNull() {
        assertNull(Main.findClosestVertex(new Vector3f(0f, 0f, 0f), Arrays.asList()));
    }

    @Test
    void findClosestVertex_singleVertex_returnsThatVertex() {
        Vertex v = vertexAt(3f, 0f, 0f);
        assertEquals(v, Main.findClosestVertex(new Vector3f(0f, 0f, 0f), Arrays.asList(v)));
    }

    @Test
    void findClosestVertex_exactMatch_returnsThatVertex() {
        Vertex v1 = vertexAt(0f, 0f, 0f);
        Vertex v2 = vertexAt(1f, 0f, 0f);

        assertEquals(v2, Main.findClosestVertex(new Vector3f(1f, 0f, 0f), Arrays.asList(v1, v2)));
    }
}
