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
