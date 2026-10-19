plugins {
    application
    id("org.openjfx.javafxplugin") version "0.1.0"
}

javafx {
    version = "21.0.12"
    modules = listOf("javafx.controls", "javafx.swing")
}

dependencies {
    implementation(project(":sim-core"))
}

application {
    mainClass.set("tunnelsim.app.Main")
}

tasks.named<JavaExec>("run") {
    workingDir = rootProject.projectDir
}

dependencies {
    // Pure-Java H.264/MP4 encoder, used only by the --record mode.
    implementation("org.jcodec:jcodec-javase:0.2.5")
}
