plugins {
    `java-library`
}

dependencies {
    api("com.fasterxml.jackson.dataformat:jackson-dataformat-yaml:2.21.7")
    api("com.fasterxml.jackson.core:jackson-databind:2.21.7")
}

tasks.withType<Test>().configureEach {
    maxHeapSize = "2g"
}
