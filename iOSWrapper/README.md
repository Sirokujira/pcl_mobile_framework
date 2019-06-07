# pcl_mobile_framework

[![Swift 4.2](https://img.shields.io/badge/Swift-4.2-orange.svg?style=flat)](https://swift.org/)
[![Version](https://img.shields.io/cocoapods/v/pcl_mobile_framework.svg?style=flat)](http://cocoapods.org/pods/pcl_mobile_framework)
[![License](https://img.shields.io/cocoapods/l/pcl_mobile_framework.svg?style=flat)](http://cocoapods.org/pods/pcl_mobile_framework)
[![Platform](https://img.shields.io/cocoapods/p/pcl_mobile_framework.svg?style=flat)](http://cocoapods.org/pods/pcl_mobile_framework)
[![Carthage compatible](https://img.shields.io/badge/Carthage-compatible-4BC51D.svg?style=flat)](https://github.com/Carthage/Carthage)
[![CI Status](http://img.shields.io/travis/Sirokujira/pcl_mobile_framework.svg?style=flat)](https://travis-ci.org/Sirokujira/pcl_mobile_framework)
[![Azure CI Status](http://img.shields.io/travis/Sirokujira/pcl_mobile_framework.svg?style=flat)](https://microsoft.com/Sirokujira/pcl_mobile_framework)

Swift implementation of a k-dimensional binary space partitioning tree.

## Usage

Import the package in your *.swift file:
```swift
import pcl
```

Make sure your data values conforom to 
```swift
public protocol pcl_mobile_frameworkPoint: Equatable {
  static var dimensions: Int { get }
  func kdDimension(dimension: Int) -> Double
  func squaredDistance(otherPoint: Self) -> Double
}
```
(CGPoint conforms to pcl_mobile_frameworkPoint as part of the package)

Then you can grow your own Tree:
```swift
extension CustomDataPoint: pcl_mobile_frameworkPoint { ... }

let dataValues: [CustomDataPoint] = ...

var tree: pcl_mobile_framework<CGPoint> = pcl_mobile_framework(values: dataValues)
```

Then you can `insert()`, `remove()`, `map()`, `filter()`, `reduce()` and `forEach()` on this tree with the expected results, as pcl_mobile_framework conforms to Sequence.

## Applications

### Feature:

Given a pcl_mobile_framework:

we can retrieve the nearest Neighbour to a test point like so
```swift
let nearest: CGPoint? = tree.nearest(to: point)
```

or the get the 10 nearest neighbours

```swift
let nearestPoints: [CGPoint] = tree.nearestK(10, to: point)
```

Complexity is O(log N), while brute-force searching through an Array is of cource O(N).

Preliminary performance results can be gained by running the unit tests, the load example has 10.000 random points in [-1,1]x[-1,1] and find the nearest points for 500 test points:

![Performance Results](/Screenshots/performance.png?raw=true)


### Filter:
![Filter Example](/Screenshots/tesselations.png?raw=true)

### Geometry:
![Geometry Example](/Screenshots/tesselations.png?raw=true)

### KdTree:
![KdTree Example](/Screenshots/tesselations.png?raw=true)

### Keypoints:
![Keypoints Example](/Screenshots/tesselations.png?raw=true)

### Octree:
![Octree Example](/Screenshots/tesselations.png?raw=true)

### People:
![People Example](/Screenshots/tesselations.png?raw=true)

### RangeImages:
![RangeImages Example](/Screenshots/tesselations.png?raw=true)

### Recognition:
![Recognition Example](/Screenshots/tesselations.png?raw=true)

### Registration:
![Registration Example](/Screenshots/tesselations.png?raw=true)

### SampleConsensus:
![SampleConsensus Example](/Screenshots/tesselations.png?raw=true)

### Search:
![Search Example](/Screenshots/tesselations.png?raw=true)

### Segmentation:
![Segmentation Example](/Screenshots/tesselations.png?raw=true)

### Stereo:
![Stereo Example](/Screenshots/tesselations.png?raw=true)

### Surface:
![Surface Example](/Screenshots/tesselations.png?raw=true)

### Tracking:
![Tracking Example](/Screenshots/tesselations.png?raw=true)


## Installation

#### Cocoapods

pcl_mobile_framework is available through [CocoaPods](http://cocoapods.org). To install
it, simply add the following line to your Podfile:

```ruby
pod "pcl_mobile_framework"
```


To run the example project, clone the repo, and run `pod install` from the Example directory first.

--- 

#### Swift package manager

Add the following to your `Package.swift` dependencies

```
.Package(url: "https://github.com/Sirokujira/pcl_mobile_framework", majorVersion: 0, minor: 0),
```

---

#### Carthage

To add `pcl_mobile_framework` using Carthage add the following to your Cartfile:

```
github "Sirokujira/pcl_mobile_framework"
```

## License

pcl_mobile_framework is available under the MIT license. See the LICENSE file for more info.
