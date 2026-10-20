package tunnelsim.app;

import javafx.scene.paint.Color;

/** Fixed colours so each estimator looks the same in every view. */
final class Palette {
    static final Color BACKGROUND = Color.web("#f7f7f5");
    static final Color PANEL = Color.web("#ffffff");
    static final Color GRID = Color.web("#e4e4e0");
    static final Color TEXT = Color.web("#2b2b2b");
    static final Color MUTED = Color.web("#8a8a85");
    static final Color ROAD = Color.web("#b9b9b4");
    static final Color ROAD_EDGE = Color.web("#9a9a95");
    static final Color TUNNEL = Color.web("#4a4a48");
    static final Color TRUTH = Color.web("#111111");
    static final Color GNSS = Color.web("#d94f70");
    static final Color BEACON = Color.web("#2a9d8f");

    private Palette() {
    }

    /** Colour of each estimator (orange A, purple B, blue C, green D, teal D+E). */
    static Color estimator(String id) {
        return switch (id) {
            case "A" -> Color.web("#e07a1f");
            case "B" -> Color.web("#8e5cc4");
            case "C" -> Color.web("#2f6fd1");
            case "D" -> Color.web("#2e9e4f");
            case "D+E" -> Color.web("#0f8b8d");
            default -> Color.GRAY;
        };
    }

    /** The same colour with a given opacity (0 transparent, 1 opaque). */
    static Color withAlpha(Color c, double a) {
        return new Color(c.getRed(), c.getGreen(), c.getBlue(), a);
    }
}
