#include "layout_prediction/settings.h"

namespace MYSLAM {
    int SIMULATION;
    int SLAM_ONLINE;
    string PCL_FRAME;
    int WALL_DETECTOR_METHOD;
    int WALL_DETECTOR_CLOUD_ROW;
    double DATA_ASSOCIATION_THRESHOLD;
    int SIM_NUMBER_OF_ITERATIONS;
    int TRACKER_METHOD;
    int PF_NUMBER_OF_PARTICLES;

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
          cfg.lookupValue ("data_association_threshold", DATA_ASSOCIATION_THRESHOLD);
          cfg.lookupValue ("sim_number_of_iterations", SIM_NUMBER_OF_ITERATIONS);
          cfg.lookupValue ("tracker_method", TRACKER_METHOD);
          cfg.lookupValue ("pf_number_of_particles", PF_NUMBER_OF_PARTICLES);
        } catch (SettingNotFoundException &nfex) {
            cerr << "Configuration not found!" << endl;
        }
    }
}
