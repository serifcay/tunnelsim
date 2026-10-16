package tunnelsim.app;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.Pane;

/** A pane holding a canvas that follows the pane's size and redraws on resize. */
abstract class CanvasPane extends Pane {
    protected final Canvas canvas = new Canvas();

    CanvasPane() {
        getChildren().add(canvas);
        canvas.widthProperty().bind(widthProperty());
        canvas.heightProperty().bind(heightProperty());
        canvas.widthProperty().addListener(o -> redraw());
        canvas.heightProperty().addListener(o -> redraw());
        setMinSize(50, 50);
    }

    protected double w() {
        return canvas.getWidth();
    }

    protected double h() {
        return canvas.getHeight();
    }

    protected GraphicsContext g() {
        return canvas.getGraphicsContext2D();
    }

    abstract void redraw();
}
