plugins {
    application
}

dependencies {
    implementation(project(":sim-core"))
    implementation("org.knowm.xchart:xchart:3.8.8")
}

application {
    mainClass.set("tunnelsim.batch.BatchMain")
}

tasks.named<JavaExec>("run") {
    workingDir = rootProject.projectDir
}

tasks.named<JavaExec>("run") {
    maxHeapSize = "2g"
}
