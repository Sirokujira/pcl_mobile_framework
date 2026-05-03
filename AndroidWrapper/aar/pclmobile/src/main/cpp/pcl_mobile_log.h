#ifndef PCL_MOBILE_LOG_H
#define PCL_MOBILE_LOG_H

#include <android/log.h>

#define LOG_TAG "libpclmobile"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

#endif // PCL_MOBILE_LOG_H
