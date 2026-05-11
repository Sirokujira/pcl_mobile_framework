import 'dart:async';
import 'dart:io';

import 'package:flutter/services.dart';
import 'package:path_provider/path_provider.dart';

class PclMobile {
  static const MethodChannel _channel = MethodChannel('pcl_mobile');

  Future<void> load(String pcdPath) {
    return _channel.invokeMethod<void>('load', {'path': pcdPath});
  }

  Future<List<double>> getCloudPoints() {
    return _doubleList('getCloudPoints');
  }

  Future<List<double>> getFilteredPoints() {
    return _doubleList('getFilteredPoints');
  }

  Future<BoundsResult> computeCentroidAndBounds() async {
    final values = await _doubleList('computeCentroidAndBounds');
    if (values.length < 10) {
      throw StateError(
        'Expected 10 centroid/bounds values, got ${values.length}.',
      );
    }
    return BoundsResult(values);
  }

  Future<void> filterVoxelGrid(double x, double y, double z) {
    return _channel.invokeMethod<void>('filterVoxelGrid', {
      'x': x,
      'y': y,
      'z': z,
    });
  }

  Future<PclMobileRunResult> runVoxelGridSample() async {
    final file = await writeSamplePcd();
    await load(file.path);
    final rawPoints = await getCloudPoints();
    final before = await computeCentroidAndBounds();

    await filterVoxelGrid(0.14, 0.14, 0.14);
    final filteredPoints = await getFilteredPoints();
    final after = await computeCentroidAndBounds();

    return PclMobileRunResult(
      pcdPath: file.path,
      rawPointCount: rawPoints.length ~/ 3,
      filteredPointCount: filteredPoints.length ~/ 3,
      filteredPoints: filteredPoints,
      before: before,
      after: after,
    );
  }

  static Future<File> writeSamplePcd() async {
    final directory = await getTemporaryDirectory();
    final file = File('${directory.path}/pclmobile_flutter_sample.pcd');
    final points = <String>[];

    for (var ix = 0; ix < 32; ix++) {
      final x = -0.8 + ix * 0.05;
      for (var iy = 0; iy < 24; iy++) {
        final y = -0.6 + iy * 0.05;
        final z = 0.20 + ((ix % 5) * 0.025) + ((iy % 7) * 0.015);
        points.add('${_f(x)} ${_f(y)} ${_f(z)}');
      }
    }

    final body = points.join('\n');
    await file.writeAsString('''
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH ${points.length}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS ${points.length}
DATA ascii
$body
''');
    return file;
  }

  static String _f(double value) => value.toStringAsFixed(4);

  static Future<List<double>> _doubleList(String method) async {
    final result = await _channel.invokeMethod<List<dynamic>>(method);
    if (result == null) {
      return const <double>[];
    }
    return result.cast<num>().map((value) => value.toDouble()).toList();
  }
}

class BoundsResult {
  BoundsResult(List<double> values)
    : centroidX = values[0],
      centroidY = values[1],
      centroidZ = values[2],
      minX = values[3],
      minY = values[4],
      minZ = values[5],
      maxX = values[6],
      maxY = values[7],
      maxZ = values[8],
      pointCount = values[9].round();

  final double centroidX;
  final double centroidY;
  final double centroidZ;
  final double minX;
  final double minY;
  final double minZ;
  final double maxX;
  final double maxY;
  final double maxZ;
  final int pointCount;
}

class PclMobileRunResult {
  const PclMobileRunResult({
    required this.pcdPath,
    required this.rawPointCount,
    required this.filteredPointCount,
    required this.filteredPoints,
    required this.before,
    required this.after,
  });

  final String pcdPath;
  final int rawPointCount;
  final int filteredPointCount;
  final List<double> filteredPoints;
  final BoundsResult before;
  final BoundsResult after;
}
