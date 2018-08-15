#include "myslam_sim_gazebo/settings.h"

namespace MYSLAM {
    int KEYFRAME_POINT;
    int OPTIMIZATION_POINT;
    double A1;
    double A2;
    double A3;
    double A4;
    double DIST_NOISE;
    double THETA_NOISE;

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
          cfg.lookupValue ("keyframe_point", KEYFRAME_POINT);
          cfg.lookupValue ("optimization_point", OPTIMIZATION_POINT);
          cfg.lookupValue ("a1", A1);
          cfg.lookupValue ("a2", A2);
          cfg.lookupValue ("a3", A3);
          cfg.lookupValue ("a4", A4);
          cfg.lookupValue ("dist_noise", DIST_NOISE);
          cfg.lookupValue ("theta_noise", THETA_NOISE);
        } catch (SettingNotFoundException &nfex) {
            cerr << "Configuration not found!" << endl;
        }
    }
}
