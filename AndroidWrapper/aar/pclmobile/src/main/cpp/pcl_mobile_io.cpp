#include "pcl_mobile_io.h"

#include <pcl/io/ifs_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>

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

void loadPLYFile(const std::string& filename)
{
    clearFilteredCloud();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud()) == -1) {
        cloud()->clear();
        LOGE("Failed to load PLY file: %s", filename.c_str());
        return;
    }

    LOGI("Loaded PLY file: %s points=%zu", filename.c_str(), cloud()->points.size());
}

void loadOBJFile(const std::string& filename)
{
    clearFilteredCloud();
    if (pcl::io::loadOBJFile<pcl::PointXYZ>(filename, *cloud()) < 0) {
        cloud()->clear();
        LOGE("Failed to load OBJ file: %s", filename.c_str());
        return;
    }

    LOGI("Loaded OBJ file: %s points=%zu", filename.c_str(), cloud()->points.size());
}

void loadIFSFile(const std::string& filename)
{
    clearFilteredCloud();
    if (pcl::io::loadIFSFile<pcl::PointXYZ>(filename, *cloud()) < 0) {
        cloud()->clear();
        LOGE("Failed to load IFS file: %s", filename.c_str());
        return;
    }

    LOGI("Loaded IFS file: %s points=%zu", filename.c_str(), cloud()->points.size());
}

void setCloudFromPackedXYZ(const std::vector<jfloat>& packed_xyz)
{
    clearFilteredCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloud();
    target->clear();
    target->points.reserve(packed_xyz.size() / 3);
    for (std::size_t i = 0; i + 2 < packed_xyz.size(); i += 3) {
        target->points.emplace_back(packed_xyz[i], packed_xyz[i + 1], packed_xyz[i + 2]);
    }
    target->width = static_cast<std::uint32_t>(target->points.size());
    target->height = 1;
    target->is_dense = false;
    LOGI("Loaded packed XYZ cloud: points=%zu ignored_values=%zu",
         target->points.size(), packed_xyz.size() % 3);
}

bool writePCDFileASCII(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
{
    if (filename.empty() || source == nullptr || source->empty()) {
        LOGE("Refused to write empty PCD file target=%s points=%zu",
             filename.c_str(), source == nullptr ? 0 : source->points.size());
        return false;
    }
    const bool ok = pcl::io::savePCDFileASCII(filename, *source) == 0;
    LOGI("Saved PCD file: %s ok=%d points=%zu", filename.c_str(), ok ? 1 : 0, source->points.size());
    return ok;
}

bool writePLYFileASCII(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
{
    if (filename.empty() || source == nullptr || source->empty()) {
        LOGE("Refused to write empty PLY file target=%s points=%zu",
             filename.c_str(), source == nullptr ? 0 : source->points.size());
        return false;
    }
    const bool ok = pcl::io::savePLYFileASCII(filename, *source) == 0;
    LOGI("Saved PLY file: %s ok=%d points=%zu", filename.c_str(), ok ? 1 : 0, source->points.size());
    return ok;
}

bool writeIFSFile(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source)
{
    if (filename.empty() || source == nullptr || source->empty()) {
        LOGE("Refused to write empty IFS file target=%s points=%zu",
             filename.c_str(), source == nullptr ? 0 : source->points.size());
        return false;
    }
    const bool ok = pcl::IFSWriter().write<pcl::PointXYZ>(filename, *source, "pclmobile_cloud") == 0;
    LOGI("Saved IFS file: %s ok=%d points=%zu", filename.c_str(), ok ? 1 : 0, source->points.size());
    return ok;
}

} // namespace pclmobile
