// Switch stand-in for jni.h: there is no JVM, only the handle types that the
// shared frontend headers mention are declared.
#pragma once

#include <stdint.h>

typedef uint8_t jboolean;
typedef int8_t jbyte;
typedef uint16_t jchar;
typedef int16_t jshort;
typedef int32_t jint;
typedef int64_t jlong;
typedef float jfloat;
typedef double jdouble;
typedef jint jsize;

class _jobject {};
typedef _jobject* jobject;
typedef jobject jclass;
typedef jobject jstring;
typedef jobject jarray;
typedef jobject jobjectArray;
typedef jobject jbyteArray;
typedef jobject jintArray;
typedef jobject jlongArray;
typedef jobject jthrowable;
typedef jobject jweak;

struct _jmethodID;
typedef _jmethodID* jmethodID;
struct _jfieldID;
typedef _jfieldID* jfieldID;

struct JNIEnv;
struct JavaVM;
