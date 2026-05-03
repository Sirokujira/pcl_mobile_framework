#ifndef PCL_MOBILE_IO_H
#define PCL_MOBILE_IO_H

#include <jni.h>

#include <string>

namespace pclmobile {

std::string jstringToString(JNIEnv* env, jstring value);
void loadPCDFile(const std::string& filename);

} // namespace pclmobile

#endif // PCL_MOBILE_IO_H
