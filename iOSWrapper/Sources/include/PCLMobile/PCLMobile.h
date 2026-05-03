// PCLMobile.h
// Umbrella header for the PCLMobile framework.
//
// Importing <PCLMobile/PCLMobile.h> from Swift, Objective-C, or
// Objective-C++ pulls in the public Objective-C interface and **does not**
// expose any of the underlying C++ types (PCL, Boost, Eigen, FLANN, Qhull).
// Consumers therefore don't need to flip their .m files to .mm.

#import <Foundation/Foundation.h>

//! Project version number for PCLMobile.
FOUNDATION_EXPORT double PCLMobileVersionNumber;

//! Project version string for PCLMobile.
FOUNDATION_EXPORT const unsigned char PCLMobileVersionString[];

#import <PCLMobile/PCLMPointCloud.h>
