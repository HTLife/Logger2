/*
 * Logger2.h
 *
 *  Created on: 15 Jun 2012
 *      Author: thomas
 */

/**
 * Format is:
 * int32_t at file beginning for frame count
 *
 * For each frame:
 * int64_t: timestamp
 * int32_t: depthSize
 * int32_t: imageSize
 * depthSize * unsigned char: depth_compress_buf
 * imageSize * unsigned char: encodedImage->data.ptr
 */

#ifndef LOGGER2_H_
#define LOGGER2_H_

#include <zlib.h>

#include <limits>
#include <cassert>
#include <iostream>

#include <opencv2/opencv.hpp>

#ifndef Q_MOC_RUN
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/condition_variable.hpp>

#include "OpenNI2/OpenNI2Interface.h"
#include "MemoryBuffer.h"
#include "TcpHandler.h"
#endif


#include "mscl/Types.h"

#include "mscl/Communication/Connection.h"
#include "mscl/MicroStrain/Inertial/InertialNode.h"
#include "mscl/Exceptions.h"
#include "mscl/MicroStrain/MIP/MipTypes.h"

#include "common.h"
#include "common_3d.h"
#include "JPEGLoader.h"
#include <zlib.h>

#include <fstream>
#include <sstream>

class IMURecord
{
public:
    IMURecord()
    {
        this->timestamp = 0;
        this->ax = 0;
        this->ay = 0;
        this->az = 0;
        this->wx = 0;
        this->wy = 0;
        this->wz = 0;
    }

    void setTime(int64_t timestamp)
    {
        this->timestamp = timestamp;
    }

    void setRecord(double ax,
                   double ay,
                   double az,
                   double wx,
                   double wy,
                   double wz)
    {
        this->ax = ax;
        this->ay = ay;
        this->az = az;
        this->wx = wx;
        this->wy = wy;
        this->wz = wz;
    }

    std::string getStr() {
        std::string s = "";

        std::ostringstream strs;
        strs << timestamp << ", "
             << wx << ", "
             << wy << ", "
             << wz << ", "
             << ax << ", "
             << ay << ", "
             << az;
        s = strs.str();
        //std::cout << s << std::endl << std::endl;
        return s;
    }
private:
    int64_t timestamp;
    double ax;
    double ay;
    double az;

    double wx;
    double wy;
    double wz;
};

class IMURecordList
{
public:
    void append(IMURecord record){
        this->vIMU_records.push_back(record);
    }
    void clear(){
        this->vIMU_records.clear();
    }
    void save(std::string savepath)
    {
        std::cout << "Format time,wx,wy,wz,ax,ay,az" << std::endl;
        std::cout << "Amount of records: "
                  << this->vIMU_records.size() << std::endl;

        std::ofstream myfile;
        myfile.open (savepath, std::ios::out);
        std::vector<IMURecord>::iterator it;
        for(it= this->vIMU_records.begin();
            it!=this->vIMU_records.end();
            it++)
        {
            myfile << it->getStr() << "\n";
        }
        myfile.close();
    }
private:
    std::vector<IMURecord> vIMU_records;
};

class Logger2
{
    public:
        Logger2(int width, int height, int fps, bool tcp, std::string logFolder);
        virtual ~Logger2();

        void startWriting(std::string filename);
        void stopWriting(QWidget * parent);

        OpenNI2Interface * getOpenNI2Interface()
        {
            return openNI2Interface;
        }

        MemoryBuffer & getMemoryBuffer()
        {
            return memoryBuffer;
        }

        void setMemoryRecord(bool value)
        {
            assert(!writing.getValue());

            logToMemory = value;
        }

        void setCompressed(bool value)
        {
            assert(!writing.getValue());

            compressed = value;
        }

        ThreadMutexObject<std::pair<bool, int64_t> > dropping;

    private:
        OpenNI2Interface * openNI2Interface;
        MemoryBuffer memoryBuffer;

        int depth_compress_buf_size;
        uint8_t * depth_compress_buf;
        CvMat * encodedImage;

        int lastWritten;
        boost::thread * writeThread;
        boost::thread * imuThread;//imu
        ThreadMutexObject<bool> writing;

        boost::mutex mutex_imu;//imu
        IMURecord current_record;

        std::string logFolder;
        std::string filename;
        int64_t lastTimestamp;

        int width;
        int height;
        int fps;

        TcpHandler * tcp;
        uint8_t * tcpBuffer;

        void encodeJpeg(cv::Vec<unsigned char, 3> * rgb_data);
        void loggingThread();
        void imuLogThread();

        FILE * logFile;
        int32_t numFrames;

        bool logToMemory;
        bool compressed;

        void logData(int64_t * timestamp,
                     int32_t * depthSize,
                     int32_t * imageSize,
                     unsigned char * depthData,
                     unsigned char * rgbData);

        void klgToPng(std::string inputFile,
                      std::string outputDir);

        IMURecordList imuRecordList;

};

#endif /* LOGGER2_H_ */
