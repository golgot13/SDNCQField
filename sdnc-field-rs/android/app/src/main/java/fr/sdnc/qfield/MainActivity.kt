package fr.sdnc.qfield

import android.Manifest
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import androidx.core.app.ActivityCompat
import com.google.androidgamesdk.GameActivity

class MainActivity : GameActivity(), LocationListener {

    companion object {
        init {
            System.loadLibrary("sdnc_field_android")
        }
    }

    private external fun nativeOnLocationUpdate(
        lat: Double,
        lon: Double,
        altEllipsoid: Double,
        accuracyM: Float,
        numSats: Int
    )

    private var locationManager: LocationManager? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        locationManager = getSystemService(LOCATION_SERVICE) as? LocationManager
        requestGnssUpdatesIfPermitted()
    }

    private fun requestGnssUpdatesIfPermitted() {
        val fineGranted = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        if (!fineGranted) {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
                1001
            )
            return
        }
        locationManager?.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            500L,
            0.0f,
            this
        )
    }

    override fun onLocationChanged(location: Location) {
        val sats = location.extras?.getInt("satellites", 10) ?: 10
        nativeOnLocationUpdate(
            location.latitude,
            location.longitude,
            location.altitude,
            location.accuracy,
            sats
        )
    }
}
