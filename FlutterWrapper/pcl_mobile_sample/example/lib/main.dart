import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:pcl_mobile_flutter/pcl_mobile_flutter.dart';

void main() {
  runApp(const PclMobileSampleApp());
}

class PclMobileSampleApp extends StatelessWidget {
  const PclMobileSampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'pclMobile Flutter Sample',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.teal),
        useMaterial3: true,
      ),
      home: const PclMobileSamplePage(),
    );
  }
}

class PclMobileSamplePage extends StatefulWidget {
  const PclMobileSamplePage({super.key});

  @override
  State<PclMobileSamplePage> createState() => _PclMobileSamplePageState();
}

class _PclMobileSamplePageState extends State<PclMobileSamplePage> {
  final PclMobile _pclMobile = PclMobile();

  bool _running = false;
  String? _error;
  PclMobileRunResult? _result;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _runSample();
    });
  }

  Future<void> _runSample() async {
    setState(() {
      _running = true;
      _error = null;
      _result = null;
    });

    try {
      final result = await _pclMobile.runVoxelGridSample();
      debugPrint(
        'pclMobile Flutter sample completed: raw=${result.rawPointCount}, '
        'voxel=${result.filteredPointCount}',
      );
      if (!mounted) return;
      setState(() {
        _result = result;
      });
    } catch (error) {
      debugPrint('pclMobile Flutter sample failed: $error');
      if (!mounted) return;
      setState(() {
        _error = error.toString();
      });
    } finally {
      if (!mounted) return;
      setState(() {
        _running = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    final result = _result;
    return Scaffold(
      appBar: AppBar(title: const Text('pclMobile Flutter Sample')),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          FilledButton(
            onPressed: _running ? null : _runSample,
            child: Text(_running ? 'Running...' : 'Run VoxelGrid sample'),
          ),
          const SizedBox(height: 16),
          if (_error != null)
            SelectableText(
              _error!,
              style: TextStyle(color: Theme.of(context).colorScheme.error),
            ),
          if (result != null) ...[
            PointCloudViewport(
              points: result.filteredPoints,
              bounds: result.after,
            ),
            const SizedBox(height: 16),
            _MetricRow(label: 'PCD path', value: result.pcdPath),
            _MetricRow(label: 'Raw points', value: '${result.rawPointCount}'),
            _MetricRow(
              label: 'VoxelGrid points',
              value: '${result.filteredPointCount}',
            ),
            _MetricRow(
              label: 'Centroid before',
              value: _formatXYZ(
                result.before.centroidX,
                result.before.centroidY,
                result.before.centroidZ,
              ),
            ),
            _MetricRow(
              label: 'Centroid after',
              value: _formatXYZ(
                result.after.centroidX,
                result.after.centroidY,
                result.after.centroidZ,
              ),
            ),
            _MetricRow(
              label: 'Bounds after',
              value:
                  '${_formatXYZ(result.after.minX, result.after.minY, result.after.minZ)} -> '
                  '${_formatXYZ(result.after.maxX, result.after.maxY, result.after.maxZ)}',
            ),
          ],
        ],
      ),
    );
  }

  static String _formatXYZ(double x, double y, double z) {
    return '(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
  }
}

class PointCloudViewport extends StatelessWidget {
  const PointCloudViewport({
    super.key,
    required this.points,
    required this.bounds,
  });

  final List<double> points;
  final BoundsResult bounds;

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Point cloud view',
          style: Theme.of(context).textTheme.titleMedium,
        ),
        const SizedBox(height: 8),
        AspectRatio(
          aspectRatio: 1.35,
          child: DecoratedBox(
            decoration: BoxDecoration(
              color: const Color(0xFF0F172A),
              border: Border.all(color: const Color(0xFF334155)),
              borderRadius: BorderRadius.circular(8),
            ),
            child: CustomPaint(
              painter: PointCloudPainter(
                points: points,
                bounds: bounds,
              ),
            ),
          ),
        ),
        const SizedBox(height: 8),
        Text(
          '${points.length ~/ 3} filtered points rendered',
          style: Theme.of(context).textTheme.bodySmall,
        ),
      ],
    );
  }
}

class PointCloudPainter extends CustomPainter {
  const PointCloudPainter({
    required this.points,
    required this.bounds,
  });

  final List<double> points;
  final BoundsResult bounds;

  @override
  void paint(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = const Color(0xFF1E293B)
      ..strokeWidth = 1;
    for (var i = 1; i < 4; i++) {
      final x = size.width * i / 4;
      final y = size.height * i / 4;
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }

    if (points.length < 3) {
      return;
    }

    final minX = bounds.minX;
    final maxX = bounds.maxX;
    final minY = bounds.minY;
    final maxY = bounds.maxY;
    final minZ = bounds.minZ;
    final maxZ = bounds.maxZ;
    final spanX = math.max(maxX - minX, 0.0001);
    final spanY = math.max(maxY - minY, 0.0001);
    final spanZ = math.max(maxZ - minZ, 0.0001);
    const padding = 18.0;
    final drawWidth = math.max(size.width - padding * 2, 1.0);
    final drawHeight = math.max(size.height - padding * 2, 1.0);
    final pointRadius = math.max(2.0, math.min(size.shortestSide / 80, 4.0));
    final pointPaint = Paint()..style = PaintingStyle.fill;

    for (var i = 0; i + 2 < points.length; i += 3) {
      final x = points[i];
      final y = points[i + 1];
      final z = points[i + 2];
      final px = padding + ((x - minX) / spanX) * drawWidth;
      final py = padding + (1.0 - ((y - minY) / spanY)) * drawHeight;
      final depth = ((z - minZ) / spanZ).clamp(0.0, 1.0);
      pointPaint.color = Color.lerp(
        const Color(0xFF38BDF8),
        const Color(0xFFFDE047),
        depth,
      )!;
      canvas.drawCircle(Offset(px, py), pointRadius, pointPaint);
    }
  }

  @override
  bool shouldRepaint(PointCloudPainter oldDelegate) {
    return oldDelegate.points != points || oldDelegate.bounds != bounds;
  }
}

class _MetricRow extends StatelessWidget {
  const _MetricRow({
    required this.label,
    required this.value,
  });

  final String label;
  final String value;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            label,
            style: Theme.of(context).textTheme.labelLarge,
          ),
          const SizedBox(height: 2),
          SelectableText(value),
        ],
      ),
    );
  }
}
