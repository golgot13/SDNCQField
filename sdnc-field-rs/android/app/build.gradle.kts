plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "fr.sdnc.qfield"
    compileSdk = 35

    defaultConfig {
        applicationId = "fr.sdnc.qfield"
        minSdk = 29 // Android 10+ (Vulkan 1.1 garanti sur tablettes et téléphones)
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0"
        ndk {
            // arm64-v8a (smartphones & tablettes modernes) + armeabi-v7a (appareils 32 bits)
            abiFilters += listOf("arm64-v8a", "armeabi-v7a")
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }
}

dependencies {
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("androidx.games:games-activity:3.0.5")
}
