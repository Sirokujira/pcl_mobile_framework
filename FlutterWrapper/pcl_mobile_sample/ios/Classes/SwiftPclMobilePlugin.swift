import Flutter
import PCLMobile
import UIKit

public class SwiftPclMobilePlugin: NSObject, FlutterPlugin {
    private var sourceCloud: PointCloud?
    private var filteredCloud: PointCloud?

    public static func register(with registrar: FlutterPluginRegistrar) {
        let channel = FlutterMethodChannel(
            name: "pcl_mobile",
            binaryMessenger: registrar.messenger()
        )
        let instance = SwiftPclMobilePlugin()
        registrar.addMethodCallDelegate(instance, channel: channel)
    }

    public func handle(_ call: FlutterMethodCall, result: @escaping FlutterResult) {
        do {
            switch call.method {
            case "load":
                let path = try requiredString(call.arguments, key: "path")
                sourceCloud = try PointCloud.load(pcdAt: path)
                filteredCloud = nil
                result(nil)
            case "getCloudPoints":
                result(try packedPoints(from: requiredSourceCloud()))
            case "getFilteredPoints":
                guard let cloud = filteredCloud else {
                    result([])
                    return
                }
                result(try packedPoints(from: cloud))
            case "computeCentroidAndBounds":
                result(try centroidAndBounds(from: activeCloud()))
            case "filterVoxelGrid":
                let x = try requiredDouble(call.arguments, key: "x")
                let y = try requiredDouble(call.arguments, key: "y")
                let z = try requiredDouble(call.arguments, key: "z")
                let leaf = min(x, y, z)
                filteredCloud = try requiredSourceCloud().voxelGridDownsampled(leaf: leaf)
                result(nil)
            default:
                result(FlutterMethodNotImplemented)
            }
        } catch {
            result(FlutterError(
                code: "PCL_MOBILE_ERROR",
                message: error.localizedDescription,
                details: nil
            ))
        }
    }

    private func requiredSourceCloud() throws -> PointCloud {
        guard let cloud = sourceCloud else {
            throw PclMobilePluginError.noLoadedCloud
        }
        return cloud
    }

    private func activeCloud() throws -> PointCloud {
        if let filteredCloud {
            return filteredCloud
        }
        return try requiredSourceCloud()
    }

    private func packedPoints(from cloud: PointCloud) throws -> [Double] {
        let pointCount = cloud.pointCount
        let pointCountInt = Int(pointCount)
        guard pointCount > 0 else {
            return []
        }

        var actualCount = 0
        var values = [Float](repeating: 0, count: pointCountInt * 3)
        try values.withUnsafeMutableBufferPointer { buffer in
            guard let baseAddress = buffer.baseAddress else {
                throw PclMobilePluginError.noPointBuffer
            }
            _ = try cloud.copyPackedXYZ(
                into: baseAddress,
                capacity: pointCount,
                actualCount: &actualCount
            )
        }

        let usedValueCount = actualCount * 3
        return values.prefix(usedValueCount).map { Double($0) }
    }

    private func centroidAndBounds(from cloud: PointCloud) throws -> [Double] {
        let values = try packedPoints(from: cloud)
        guard values.count >= 3 else {
            return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        }

        let pointCount = values.count / 3
        var sumX = 0.0
        var sumY = 0.0
        var sumZ = 0.0
        var minX = Double.greatestFiniteMagnitude
        var minY = Double.greatestFiniteMagnitude
        var minZ = Double.greatestFiniteMagnitude
        var maxX = -Double.greatestFiniteMagnitude
        var maxY = -Double.greatestFiniteMagnitude
        var maxZ = -Double.greatestFiniteMagnitude

        for index in stride(from: 0, to: values.count, by: 3) {
            let x = values[index]
            let y = values[index + 1]
            let z = values[index + 2]
            sumX += x
            sumY += y
            sumZ += z
            minX = min(minX, x)
            minY = min(minY, y)
            minZ = min(minZ, z)
            maxX = max(maxX, x)
            maxY = max(maxY, y)
            maxZ = max(maxZ, z)
        }

        let count = Double(pointCount)
        return [
            sumX / count,
            sumY / count,
            sumZ / count,
            minX,
            minY,
            minZ,
            maxX,
            maxY,
            maxZ,
            count,
        ]
    }

    private func requiredString(_ arguments: Any?, key: String) throws -> String {
        guard let dictionary = arguments as? [String: Any],
              let value = dictionary[key] as? String else {
            throw PclMobilePluginError.missingArgument(key)
        }
        return value
    }

    private func requiredDouble(_ arguments: Any?, key: String) throws -> Double {
        guard let dictionary = arguments as? [String: Any],
              let value = dictionary[key] else {
            throw PclMobilePluginError.missingArgument(key)
        }

        if let double = value as? Double {
            return double
        }
        if let float = value as? Float {
            return Double(float)
        }
        if let int = value as? Int {
            return Double(int)
        }
        throw PclMobilePluginError.invalidArgument(key)
    }
}

private enum PclMobilePluginError: LocalizedError {
    case missingArgument(String)
    case invalidArgument(String)
    case noLoadedCloud
    case noPointBuffer

    var errorDescription: String? {
        switch self {
        case .missingArgument(let key):
            return "Missing argument: \(key)"
        case .invalidArgument(let key):
            return "Invalid argument: \(key)"
        case .noLoadedCloud:
            return "No point cloud has been loaded."
        case .noPointBuffer:
            return "Could not allocate a point buffer."
        }
    }
}
