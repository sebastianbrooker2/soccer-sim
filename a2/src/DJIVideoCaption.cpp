#include <cassert>
#include <cstdio>
#include <string>
#include <filesystem>
#include "DJIVideoCaption.h"

std::vector<DJIVideoCaption> getVideoCaptions(const std::filesystem::path & captionPath)
{
    assert(std::filesystem::exists(captionPath));

    std::vector<DJIVideoCaption> caps;
    
    FILE* file = std::fopen(captionPath.c_str(), "r");
    if (!file) {
        return caps;
    }
    
    int frameNum;
    int h1, m1, s1, ms1, h2, m2, s2, ms2;
    int fontSize, frameCnt, diffTime;
    int year, month, day, hour, min, sec, ms_1, ms_2;
    int iso, fnum, ev, ct, focalLen;
    float shutter;
    char colorMd[50];
    float lat, lon, alt;
    
    while (true) {
        // Read frame number
        int ret = std::fscanf(file, " %d", &frameNum);
        if (ret != 1) break;
        
        // Timestamp line  
        ret = std::fscanf(file, " %d:%d:%d,%d --> %d:%d:%d,%d", 
                   &h1, &m1, &s1, &ms1, &h2, &m2, &s2, &ms2);
        if (ret != 8) {
            break;
        }
        
        // Font and frame info
        ret = std::fscanf(file, " <font size=\"%d\">FrameCnt : %d, DiffTime : %dms", 
                   &fontSize, &frameCnt, &diffTime);
        if (ret != 3) {
            break;
        }
        
        // Date/time
        ret = std::fscanf(file, " %d-%d-%d %d:%d:%d,%d,%d",
                   &year, &month, &day, &hour, &min, &sec, &ms_1, &ms_2);
        if (ret != 8) {
            break;
        }
        
        // Data line - use %[^]] to read until ] character
        ret = std::fscanf(file, " [iso : %d] [shutter : 1/%f] [fnum : %d] [ev : %d] [ct : %d] [color_md : %49[^]]] [focal_len : %d] [latitude : %f] [longtitude : %f] [altitude: %f] </font>",
                   &iso, &shutter, &fnum, &ev, &ct, colorMd, &focalLen, &lat, &lon, &alt);
        if (ret != 10) {
            break;
        }
        
        DJIVideoCaption cap;
        cap.frameNum = frameCnt;
        cap.time = h1 * 3600.0 + m1 * 60.0 + s1 + ms1 / 1000.0;
        cap.iso = iso;
        cap.shutterHz = shutter;
        cap.fnum = fnum / 100.0;
        cap.latitude = lat;
        cap.longitude = lon;
        cap.altitude = alt;
        
        caps.push_back(cap);
    }
    
    std::fclose(file);
    return caps;
}
