package tunnelsim.app;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.stage.Stage;

public class Main extends Application {
    @Override
    public void start(Stage stage) {
        stage.setTitle("Through the Tunnel");
        stage.setScene(new Scene(new Label("tunnelsim"), 800, 600));
        stage.show();
    }

    public static void main(String[] args) {
        launch(args);
    }
}
