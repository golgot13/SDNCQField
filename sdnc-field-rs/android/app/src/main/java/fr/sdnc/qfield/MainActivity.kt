package fr.sdnc.qfield

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import androidx.core.app.ActivityCompat
import com.google.androidgamesdk.GameActivity
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import java.util.concurrent.atomic.AtomicBoolean

class MainActivity : GameActivity(), LocationListener {

    companion object {
        // UUID standard Serial Port Profile (SPP RFCOMM) utilisé par Emlid Reach & u-blox Bluetooth
        private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

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

    private external fun nativeOnExternalGnssBytes(data: ByteArray)
    private external fun nativeOnRtcmBytes(data: ByteArray)
    private external fun nativePollOutgoingTxBytes(): ByteArray?

    private var locationManager: LocationManager? = null
    private var btSocket: BluetoothSocket? = null
    private var btOutStream: OutputStream? = null
    private val btRunning = AtomicBoolean(false)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        locationManager = getSystemService(LOCATION_SERVICE) as? LocationManager
        requestPermissionsAndStartGnss()
    }

    override fun onDestroy() {
        disconnectBluetoothGnss()
        super.onDestroy()
    }

    private fun requestPermissionsAndStartGnss() {
        val perms = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            perms.add(Manifest.permission.BLUETOOTH_CONNECT)
            perms.add(Manifest.permission.BLUETOOTH_SCAN)
        }
        val missing = perms.filter {
            ActivityCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) {
            ActivityCompat.requestPermissions(this, missing.toTypedArray(), 1001)
            return
        }

        locationManager?.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            500L,
            0.0f,
            this
        )
        autoConnectPairedExternalGnss()
    }

    /// Recherche un récepteur externe appairé (Leica, Trimble, Sokkia/Topcon, SparkFun, u-blox ZED-F9P,
    /// Emlid Reach L1/M2/RS2+/RS3/RS4 Pro, Septentrio, ArduSimple) et ouvre une liaison RFCOMM SPP bidirectionnelle.
    private fun autoConnectPairedExternalGnss() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED
        ) {
            return
        }
        val btManager = getSystemService(BLUETOOTH_SERVICE) as? BluetoothManager ?: return
        val adapter: BluetoothAdapter = btManager.adapter ?: return
        if (!adapter.isEnabled) return

        val keywords = listOf(
            "reach", "emlid", "rs4", "rs3", "rs2",
            "leica", "gs18", "gs16", "gs15", "gs07", "flx100",
            "trimble", "r12", "r10", "r8s", "da2", "catalyst",
            "sokkia", "gcx", "grx", "gsx", "topcon", "hiper",
            "sparkfun", "facet", "torch", "express",
            "ublox", "u-blox", "zed-f9p", "f9p", "ardusimple", "septentrio", "gnss", "rtk"
        )
        val target: BluetoothDevice = adapter.bondedDevices.firstOrNull { dev ->
            val name = dev.name?.lowercase() ?: ""
            keywords.any { kw -> name.contains(kw) }
        } ?: return

        connectBluetoothSpp(target)
    }

    fun connectBluetoothSpp(device: BluetoothDevice) {
        disconnectBluetoothGnss()
        btRunning.set(true)
        Thread {
            try {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                    ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED
                ) {
                    return@Thread
                }
                val socket = device.createRfcommSocketToServiceRecord(SPP_UUID)
                socket.connect()
                btSocket = socket
                btOutStream = socket.outputStream
                val input = socket.inputStream
                val buf = ByteArray(2048)

                while (btRunning.get() && socket.isConnected) {
                    val tx = nativePollOutgoingTxBytes()
                    if (tx != null && tx.isNotEmpty()) {
                        btOutStream?.write(tx)
                        btOutStream?.flush()
                    }
                    if (input.available() > 0) {
                        val n = input.read(buf)
                        if (n > 0) {
                            nativeOnExternalGnssBytes(buf.copyOf(n))
                        }
                    } else {
                        Thread.sleep(25)
                    }
                }
            } catch (_: IOException) {
                // Déconnexion propre en cas de coupure radio/Bluetooth
            } catch (_: InterruptedException) {
            } finally {
                disconnectBluetoothGnss()
            }
        }.start()
    }

    /// Permet d'envoyer des trames RTCM 3.3 NTRIP vers le récepteur externe Bluetooth SPP.
    fun writeRtcmToBluetooth(rtcmBytes: ByteArray) {
        try {
            btOutStream?.write(rtcmBytes)
            btOutStream?.flush()
        } catch (_: IOException) {
        }
    }

    private fun disconnectBluetoothGnss() {
        btRunning.set(false)
        try {
            btSocket?.close()
        } catch (_: IOException) {
        }
        btSocket = null
        btOutStream = null
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
