#include <jni.h>
#include <cstdio>
#include <cmath>

jobjectArray naiveMultiply(JNIEnv *env, jobjectArray matrix1, jobjectArray matrix2) {
    int   i1, j2, select;
    jsize r1, c1, r2, c2;

    r1 = env->GetArrayLength(matrix1);
    r2 = env->GetArrayLength(matrix2);

    if (r1 == 0 || r2 == 0)
        return nullptr;

    jclass doubleArray = env->GetObjectClass(env->GetObjectArrayElement(matrix1, 0));

    c1 = env->GetArrayLength(reinterpret_cast<jarray>(env->GetObjectArrayElement(matrix1, 0)));
    c2 = env->GetArrayLength(reinterpret_cast<jarray>(env->GetObjectArrayElement(matrix2, 0)));

    if (r2 != c1)
        return nullptr;

    double sig1[r1][c1];
    double sig2[r2][c2];
    double out[r1][c2];

    for (i1 = 0; i1 < r1; i1++) {
        env->GetDoubleArrayRegion(reinterpret_cast<jdoubleArray>(env->GetObjectArrayElement(matrix1, i1)), 0, c1, sig1[i1]);
    }
    env->DeleteLocalRef(matrix1);


    for (int i2 = 0; i2 < r2; i2++) {
        env->GetDoubleArrayRegion(reinterpret_cast<jdoubleArray>(env->GetObjectArrayElement(matrix2, i2)), 0, c2, sig2[i2]);
    }
    env->DeleteLocalRef(matrix2);

    for (i1 = 0; i1 < r1; i1++) {
        for (j2 = 0; j2 < c2; j2++) {
            for (select = 0; select < r2; select++) {
                out[i1][j2] += (sig1[i1][select] * sig2[select][j2]);
            }
        }
    }

    // TODO: del     sig1,    sig2

    jobjectArray output = env->NewObjectArray(r1, doubleArray, env->NewDoubleArray(c1));


    for (i1 = 0; i1 < r1; i1++) {
        jdoubleArray element = env->NewDoubleArray(c2);
        env->SetDoubleArrayRegion(element, 0, c2, out[i1]);
        env->SetObjectArrayElement(output, i1, element);
    }

    return output;
}

jobjectArray directMultiply(JNIEnv *env, jobjectArray matrix1, jobjectArray matrix2) {
    int   i1, j2, select;
    jsize r1, c1, r2, c2;

    r1 = env->GetArrayLength(matrix1);
    r2 = env->GetArrayLength(matrix2);

    if (r1 == 0 || r2 == 0)
        return nullptr;

    c1 = env->GetArrayLength(reinterpret_cast<jarray>(env->GetObjectArrayElement(matrix1, 0)));
    c2 = env->GetArrayLength(reinterpret_cast<jarray>(env->GetObjectArrayElement(matrix2, 0)));

    if (r2 != c1)
        return nullptr;

    jdouble* sig2[r2];

    for (int i2 = 0; i2 < r2; i2++) {
        sig2[i2] = (jdouble*) env->GetObjectArrayElement(matrix1, i2);
    }

    env->DeleteLocalRef(matrix2);

    jobjectArray output = env->NewObjectArray(r1, env->FindClass("[D"), env->NewDoubleArray(c1));

    for (i1 = 0; i1 < r1; i1++) {
        double* matrix1Row = env->GetDoubleArrayElements(reinterpret_cast<jdoubleArray>(env->GetObjectArrayElement(matrix1, i1)), nullptr);

        jdoubleArray row = env->NewDoubleArray(c2);
        for (j2 = 0; j2 < c2; j2++) {
            double element = 0;
            for (select = 0; select < r2; select++) {
                element += (matrix1Row[select] * (sig2[select][j2]));
            }
            env->SetDoubleArrayRegion(row, j2, j2+1, &element);
        }
    }

    env->DeleteLocalRef(matrix1);

    // TODO: del     sig1,    sig2

    return output;
}

jdoubleArray directAdd(JNIEnv *env, jdoubleArray matrix1, jdoubleArray matrix2) {
    int   i;
    jsize c1, c2;

    c1 = env->GetArrayLength(matrix1);
    c2 = env->GetArrayLength(matrix2);

    if (c1 == 0 || c2 == 0 || c1 != c2)
        return nullptr;

    jdoubleArray output = env->NewDoubleArray(c1);

    double* matrix1Row = env->GetDoubleArrayElements(matrix1, nullptr);
    double* matrix2Row = env->GetDoubleArrayElements(matrix2, nullptr);

    for (i = 0; i < c1; i++) {
        double element = matrix1Row[i] + matrix2Row[i];
        env->SetDoubleArrayRegion(output, i, i+1, &element);
    }

    env->DeleteLocalRef(matrix1);
    env->DeleteLocalRef(matrix2);

    // TODO: del     sig1,    sig2

    return output;
}

jdoubleArray directRelu(JNIEnv *env, jdoubleArray matrix1) {
    int   i;
    jsize c1;

    c1 = env->GetArrayLength(matrix1);

    if (c1 == 0)
        return nullptr;

    jdoubleArray output = env->NewDoubleArray(c1);

    double* matrix1Row = env->GetDoubleArrayElements(matrix1, nullptr);

    for (i = 0; i < c1; i++) {
        double element = abs(matrix1Row[i]);
        env->SetDoubleArrayRegion(output, i, i+1, &element);
    }

    env->DeleteLocalRef(matrix1);

    // TODO: del     sig1,    sig2

    return output;
}

extern "C"
JNIEXPORT jobjectArray JNICALL
Java_org_firstinspires_ftc_teamcode_Matrices_multiply(JNIEnv *env, jclass thiz, jobjectArray matrix1,
                                                      jobjectArray matrix2) {
    return directMultiply(env, matrix1, matrix2);
}
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_org_firstinspires_ftc_teamcode_Matrices_add(JNIEnv *env, jclass clazz, jdoubleArray matrix1,
                                                 jdoubleArray matrix2) {
    return directAdd(env, matrix1, matrix2);
}
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_org_firstinspires_ftc_teamcode_Matrices_relu(JNIEnv *env, jclass clazz, jdoubleArray matrix1) {
    return directRelu(env, matrix1);
}