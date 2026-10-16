subprojects {
    apply(plugin = "java")

    repositories {
        mavenCentral()
    }

    // Compile for Java 21 with whatever JDK (21+) runs Gradle.
    tasks.withType<JavaCompile>().configureEach {
        options.release.set(21)
        options.encoding = "UTF-8"
        options.compilerArgs.addAll(listOf("-Xlint:all", "-Xlint:-serial", "-Xlint:-processing"))
    }

    dependencies {
        "testImplementation"(platform("org.junit:junit-bom:5.14.4"))
        "testImplementation"("org.junit.jupiter:junit-jupiter")
        "testRuntimeOnly"("org.junit.platform:junit-platform-launcher")
        "testImplementation"("org.assertj:assertj-core:3.27.7")
    }

    tasks.withType<Test>().configureEach {
        useJUnitPlatform()
    }
}
