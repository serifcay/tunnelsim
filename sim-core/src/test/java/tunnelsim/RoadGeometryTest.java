package tunnelsim;

import org.junit.jupiter.api.Test;
import tunnelsim.map.Pose;
import tunnelsim.map.RoadGeometry;
import tunnelsim.scenario.ScenarioConfig.SegmentConfig;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

class RoadGeometryTest {
    private static SegmentConfig seg(String type, double length, Double curvature, double curvatureEnd, double grade) {
        SegmentConfig s = new SegmentConfig();
        s.type = type;
        s.length = length;
        s.curvature = curvature;
        s.curvatureEnd = curvatureEnd;
        s.grade = grade;
        return s;
    }

    @Test
    void arcTurnsByCurvatureTimesLengthAndClothoidIsContinuous() {
        RoadGeometry g = RoadGeometry.build(new Pose(0, 0, 0, 0, 0, 0), List.of(
                seg("straight", 100, null, 0, 0),
                seg("clothoid", 100, null, 0.01, 0),
                seg("arc", 200, 0.01, 0, 0)), 1);
        assertThat(g.length()).isCloseTo(400, offset(0.6));
        // Heading change: clothoid 0.5*k*L = 0.5 rad, arc k*L = 2 rad.
        assertThat(g.pose(400).heading()).isCloseTo(2.5, offset(0.01));
        assertThat(g.curvature(150)).isCloseTo(0.005, offset(1e-4));
        assertThat(g.curvature(300)).isCloseTo(0.01, offset(1e-9));
    }

    @Test
    void elevationIntegratesGradeAndNearestFindsThePoint() {
        RoadGeometry g = RoadGeometry.build(new Pose(0, 0, 0, 0, 0, 0), List.of(
                seg("straight", 1000, null, 0, -0.04), seg("straight", 1000, null, 0, 0.04)), 50);
        assertThat(g.elevation(1000)).isCloseTo(-40, offset(1.5));
        assertThat(g.elevation(2000)).isCloseTo(0, offset(0.5));
        Pose p = g.pose(1234.5);
        double[] near = g.nearest(p.x() + 0.3, p.y() - 0.2, 0, g.length());
        assertThat(near[0]).isCloseTo(1234.5, offset(0.5));
    }
}
