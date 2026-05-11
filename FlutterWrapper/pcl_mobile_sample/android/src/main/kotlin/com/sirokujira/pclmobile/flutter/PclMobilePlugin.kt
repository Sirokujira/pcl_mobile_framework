package com.sirokujira.pclmobile.flutter

import com.sirokujira.pclmobile.pclmobileJNILib
import io.flutter.embedding.engine.plugins.FlutterPlugin
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import io.flutter.plugin.common.MethodChannel.MethodCallHandler
import io.flutter.plugin.common.MethodChannel.Result

class PclMobilePlugin : FlutterPlugin, MethodCallHandler {
    private lateinit var channel: MethodChannel

    override fun onAttachedToEngine(binding: FlutterPlugin.FlutterPluginBinding) {
        channel = MethodChannel(binding.binaryMessenger, "pcl_mobile")
        channel.setMethodCallHandler(this)
    }

    override fun onDetachedFromEngine(binding: FlutterPlugin.FlutterPluginBinding) {
        channel.setMethodCallHandler(null)
    }

    override fun onMethodCall(call: MethodCall, result: Result) {
        try {
            when (call.method) {
                "load" -> {
                    pclmobileJNILib.load(call.requiredString("path"))
                    result.success(null)
                }
                "getCloudPoints" -> {
                    result.success(pclmobileJNILib.getCloudPoints().toDoubleList())
                }
                "getFilteredPoints" -> {
                    result.success(pclmobileJNILib.getFilteredPoints().toDoubleList())
                }
                "computeCentroidAndBounds" -> {
                    result.success(pclmobileJNILib.computeCentroidAndBounds().toDoubleList())
                }
                "filterVoxelGrid" -> {
                    pclmobileJNILib.filterVoxelGrid(
                        call.requiredDouble("x"),
                        call.requiredDouble("y"),
                        call.requiredDouble("z"),
                    )
                    result.success(null)
                }
                else -> result.notImplemented()
            }
        } catch (error: Throwable) {
            result.error("PCL_MOBILE_ERROR", error.message, null)
        }
    }

    private fun MethodCall.requiredString(key: String): String {
        return argument<String>(key) ?: error("Missing string argument: $key")
    }

    private fun MethodCall.requiredDouble(key: String): Double {
        val value = argument<Any>(key) ?: error("Missing numeric argument: $key")
        return when (value) {
            is Double -> value
            is Float -> value.toDouble()
            is Int -> value.toDouble()
            is Long -> value.toDouble()
            else -> error("Argument $key must be numeric")
        }
    }

    private fun FloatArray.toDoubleList(): List<Double> {
        return map { it.toDouble() }
    }
}
