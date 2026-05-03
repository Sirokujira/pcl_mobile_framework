#include "pcl_mobile_io.h"

#include <pcl/io/pcd_io.h>

#include "pcl_mobile_context.h"
#include "pcl_mobile_log.h"

namespace pclmobile {

std::string jstringToString(JNIEnv* env, jstring value)
{
    if (value == nullptr) {
        return "";
    }

    const jclass string_class = env->GetObjectClass(value);
    const jmethodID get_bytes = env->GetMethodID(string_class, "getBytes", "(Ljava/lang/String;)[B");
    const jbyteArray bytes =
            static_cast<jbyteArray>(env->CallObjectMethod(value, get_bytes, env->NewStringUTF("UTF-8")));

    const size_t length = static_cast<size_t>(env->GetArrayLength(bytes));
    jbyte* raw_bytes = env->GetByteArrayElements(bytes, nullptr);

    std::string result(reinterpret_cast<char*>(raw_bytes), length);
    env->ReleaseByteArrayElements(bytes, raw_bytes, JNI_ABORT);
    env->DeleteLocalRef(bytes);
    env->DeleteLocalRef(string_class);
    return result;
}

void loadPCDFile(const std::string& filename)
{
    clearFilteredCloud();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud()) == -1) {
        cloud()->clear();
        LOGE("Failed to load PCD file: %s", filename.c_str());
        return;
    }

    LOGI("Loaded PCD file: %s points=%zu", filename.c_str(), cloud()->points.size());
}

} // namespace pclmobile
