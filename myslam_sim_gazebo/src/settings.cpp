#include "myslam_sim_gazebo/settings.h"

namespace MYSLAM {
    int KEYFRAME_POINT;
    int OPTIMIZATION_POINT;
    double A1;
    double A2;
    double A3;
    double A4;
    double B1;
    double B2;
    double B3;
    double B4;
    double B5;

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
          cfg.lookupValue ("b1", B1);
          cfg.lookupValue ("b2", B2);
          cfg.lookupValue ("b3", B3);
          cfg.lookupValue ("b4", B4);
          cfg.lookupValue ("b5", B5);
        } catch (SettingNotFoundException &nfex) {
            cerr << "Configuration not found!" << endl;
        }
    }
}
