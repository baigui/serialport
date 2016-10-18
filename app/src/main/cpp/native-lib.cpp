#include <jni.h>
#include <string>

#include <jni.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <android/log.h>

static speed_t getBaudRate(jint baudRate) {
    switch (baudRate) {
        case 0: return B0;
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:
            return -1;
    }
}

extern "C"
JNIEXPORT jobject JNICALL Java_com_baigui_demo_jni_SerialPort_open
        (JNIEnv *env, jobject obj, jstring devicePath, jint buaRate, jint flags) {
    int fd;
    speed_t speed;
    jobject mFd;
    speed   = getBaudRate(buaRate);
    if (buaRate == 115200){
//        LOGE("Invalid buaR");
    }
    if (speed == -1) {
//        LOGE("Invalid buaRate!!");
        return NULL;
    }
//    LOGI("Right buaRate = %d.", buaRate);
    /** open device */
    jboolean isCopy;
    const char *utfPath = env->GetStringUTFChars(devicePath, &isCopy);
    fd  = open(utfPath, O_RDWR | flags);
    env->ReleaseStringUTFChars(devicePath, utfPath);
    if (fd == -1) {
//        LOGE("Cannot open port");
    }
//    LOGI("Open port Success!");
    /** Configure Device*/
    struct termios cfg;
    /** 获取与该终端描述符有关的参数,结果保存在termios结构体中.成功返回0
    c_iflag:输入模式标志,控制终端输入方式.
    c_oflag:输出模式标志.
    c_cflag:控制模式标志,指定终端硬件控制信息.
    c_lflag:本地模式标志,控制终端编辑功能.
    c_cc[NCCS]:控制字符,用于保存终端驱动程序中的特殊字符,如输入结束符.
    **/
    if (tcgetattr(fd, &cfg)) {
//        LOGE("tcgetattr() failed");
        close(fd);
        return NULL;
    }
//    LOGI("tcgetattr() Success");
    /** 设置终端属性为原始属性 **/
    cfmakeraw(&cfg);
    /** 设置输入波特率 */
    cfsetispeed(&cfg, speed);
    /** 设置输出波特率 */
    cfsetospeed(&cfg, speed);
    /** 设置属性
    第二个参数表示什么时候生效.
    TCSANOW:表明该设置立即生效
    TCSADRAIN:在所有写入fd的输出都输出后生效.此参数该在参数影响输出时使用
    TCSAFLUSH:清空输入输出缓冲区才改变属性.所有写入 fd 引用的对象的输出都被传输后生效，所有已接受但未读入的输入都在改变发生前丢弃.
    **/
    if (tcsetattr(fd, TCSANOW, &cfg)) {
//        LOGE("tcsetattr() failed");
        close(fd);
        return NULL;
    }
//    LOGI("tcsetattr() Success");
    /** Create a corresponding file descriptor */
    jclass cFileDescriptor  = env->FindClass("java/io/FileDescriptor");
    jmethodID iFileDescriptor   = env->GetMethodID(cFileDescriptor, "<init>", "()V");
    jfieldID descriptorID   = env->GetFieldID(cFileDescriptor, "descriptor", "I");
    mFd = env->NewObject(cFileDescriptor, iFileDescriptor);
    env->SetIntField(mFd, descriptorID, (jint) fd);
//    LOGI("return mFd = %d.", fd);
    return mFd;
}

/*
 * Class:     com_zzx_port_SerialPort
 * Method:    close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_baigui_demo_jni_SerialPort_close
        (JNIEnv *env, jobject obj) {
    jclass SerialPortClass = env->GetObjectClass(obj);
    jclass FileDescriptorClass = env->FindClass("java/io/FileDescriptor");
    jfieldID descriptorID = env->GetFieldID(FileDescriptorClass, "descriptor", "I");

    jfieldID mFDID = env->GetFieldID(SerialPortClass, "mFd", "Ljava/io/FileDescriptor;");
    jobject mFd = env->GetObjectField(obj, mFDID);

    jint descriptor = env->GetIntField(mFd, descriptorID);
    close(descriptor);
}
