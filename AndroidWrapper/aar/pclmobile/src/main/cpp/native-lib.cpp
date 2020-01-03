// common headers
#include <jni.h>
#include <android/log.h>
#include <string>

// include pcl headers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define  LOG_TAG    "libpclmobile"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

/////
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

void callFilterAxis(std::string axis, double min, double max);
void callFilterVoxelGrid(double x, double y, double z);

//
// https://stackoverflow.com/questions/41820039/jstringjni-to-stdstringc-with-utf8-characters
// 
std::string jstring2string(JNIEnv *env, jstring jStr) {
    if (!jStr)
        return "";

    const jclass stringClass = env->GetObjectClass(jStr);
    const jmethodID getBytes = env->GetMethodID(stringClass, "getBytes", "(Ljava/lang/String;)[B");
    const jbyteArray stringJbytes = (jbyteArray) env->CallObjectMethod(jStr, getBytes, env->NewStringUTF("UTF-8"));

    size_t length = (size_t) env->GetArrayLength(stringJbytes);
    jbyte* pBytes = env->GetByteArrayElements(stringJbytes, NULL);

    std::string ret = std::string((char *)pBytes, length);
    env->ReleaseByteArrayElements(stringJbytes, pBytes, JNI_ABORT);

    env->DeleteLocalRef(stringJbytes);
    env->DeleteLocalRef(stringClass);
    return ret;
}

void loadPCDFile(std::string filename)
{
    // Fill in the cloud data
#if 0
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
#else
    // ?t?@?C????????o??
    // ????????AADM ????t?@?C????]??????????B
    // 6.0 ????Apermission ??m?F???s??????
    // ??????OK :
    // Emulator mode
    // std::string pcl_file = "storage/emulated/0/lamppost.pcd";
    // std::string pcl_file = "storage/emulated/0/" + filename;
    std::string pcl_file = filename;
    // NG(?f?[?^?T?C?Y???????H)
    // std::string pcl_file = "storage/emulated/0/bun0.pcd";
    // std::string pcl_file = "storage/emulated/0/CSite1_orig-utm.pcd";

    // /data/data/(package) folder ??t?@?C??????
    // std::string pcl_file = "lamppost.pcd";
    // ng : boost library?
    // ?????T?C?Y??ng : ?_?Q????? ??? 20k?? ???
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcl_file, *cloud) == -1) //* load the file
    {
        // PCL_ERROR ("Couldn't read file test_pcd.pcd.\n");
        return;
    }
#endif
    // callFilterAxis("y", 0.0, 0.01);
}

void callFilterAxis(std::string axis, double min, double max) {
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("z");
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (min, max);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
}

void callFilterVoxelGrid(double x, double y, double z)
{
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud (cloud);
    voxelGrid.setLeafSize(x, y, z);
    voxelGrid.filter (*cloud_filtered);
}

/////
extern "C" {
	JNIEXPORT jstring JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_stringFromJNI(JNIEnv *env, jobject /* this */);
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_init(JNIEnv * env, jobject obj,  jint width, jint height);
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_step(JNIEnv * env, jobject obj);
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_load(JNIEnv * env, jobject obj, jstring filename);
    // Feature
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_feature1(JNIEnv *env, jobject /* this */);
    // Filter
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterAxis(JNIEnv * env, jobject obj, jstring axis, jdouble minValue, jdouble maxValue);
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterVoxelGrid(JNIEnv * env, jobject obj, jdouble x, jdouble y, jdouble z);
    // Geometry
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_geometry1(JNIEnv * env, jobject obj, jstring filename);
    // KdTree
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_kdtree1(JNIEnv * env, jobject obj, jstring filename);
    // KeyPoint
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_keypoint1(JNIEnv * env, jobject obj, jstring filename);

    // Octree
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octree1(JNIEnv * env, jobject obj, jstring filename);
    // People
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_people1(JNIEnv * env, jobject obj, jstring filename);
    // RangeImages
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rangeImages1(JNIEnv * env, jobject obj, jstring filename);
    // Recognition
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_recognition1(JNIEnv * env, jobject obj, jstring filename);
    // Registration
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_registration1(JNIEnv * env, jobject obj, jstring filename);
    // SampleConsensus
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_sampleconsensus1(JNIEnv * env, jobject obj, jstring filename);
    // Segmentation
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentation1(JNIEnv * env, jobject obj, jstring filename);
    // Stereo
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_stereo1(JNIEnv * env, jobject obj, jstring filename);
    // Surface
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_surface1(JNIEnv * env, jobject obj, jstring filename);
    // Tracking
    JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_track1(JNIEnv * env, jobject obj, jstring filename);
};


JNIEXPORT jstring JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_init(JNIEnv * env, jobject obj,  jint width, jint height)
{
    // setupGraphics(width, height);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_step(JNIEnv * env, jobject obj)
{
    // renderFrame();
}

// io
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_load(JNIEnv * env, jobject obj, jstring filename)
{
	std::string c_filename = jstring2string(env, filename);
    loadPCDFile(c_filename);
}

//region Feature
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_feature1(JNIEnv * env, jobject obj, jstring filename)
{
}
//endregion

//region Filter
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterAxis(JNIEnv * env, jobject obj, jstring axis, jdouble minValue, jdouble maxValue)
{
    std::string c_axis = jstring2string(env, axis);
    callFilterAxis(c_axis, minValue, maxValue);
}

JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_filterVoxelGrid(JNIEnv * env, jobject obj, jdouble x, jdouble y, jdouble z)
{
    callFilterVoxelGrid(x, y, z);
}

//endregion

//region Geometry
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_geometry1(JNIEnv * env, jobject obj, jstring filename)
{
}

//endregion

//region KdTree
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_kdtree1(JNIEnv * env, jobject obj, jstring filename)
{
}
//endregion

//region KeyPoint
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_keypoint1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region Octree
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_octree1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region People
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_people1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region RangeImages
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_rangeimages1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region Recognition
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_recognition1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region Registration
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_registration1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region SampleConsensus
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_sampleconsensus1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region Segmentation
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_segmentation1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region Stereo
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_stereo1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region Surface
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_surface1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

//region Tracking
JNIEXPORT void JNICALL Java_com_sirokujira_pclmobile_pclmobileJNILib_tracking1(JNIEnv * env, jobject obj, jstring filename)
{

}
//endregion

