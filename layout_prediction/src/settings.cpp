#include "layout_prediction/settings.h"

namespace MYSLAM {
    int SIMULATION;
    int SLAM_ONLINE;
    string PCL_FRAME;
    int WALL_DETECTOR_METHOD;
    int WALL_DETECTOR_CLOUD_ROW;

    void loadConfFile(const char* fileconfig)
    {
        libconfig::Config cfg;
        try {
          cfg.readFile (fileconfig);
        } catch (const FileIOException &fioex) {
            cerr << "I/O error while reading file." << endl;
        } catch (const ParseException &pex) {
            cerr << "Parse error at " << pex.getFile() << ": " << pex.getLine()
                << " - " << pex.getError() << endl;
        }

        try {
          cfg.lookupValue ("simulation", SIMULATION);
          cfg.lookupValue ("online", SLAM_ONLINE);
          cfg.lookupValue ("pcl_frame", PCL_FRAME);
          cfg.lookupValue ("wall_detector_method", WALL_DETECTOR_METHOD);
          cfg.lookupValue ("wall_detector_cloud_row", WALL_DETECTOR_CLOUD_ROW);
        } catch (SettingNotFoundException &nfex) {
            cerr << "Configuration not found!" << endl;
        }
    }
}
