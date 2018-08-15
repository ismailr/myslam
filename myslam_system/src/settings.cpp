#include "myslam_system/settings.h"

namespace MYSLAM {
    double TPPF1;
    double TPPF2;
    double TPPF3;
    double TPPF4;

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
          cfg.lookupValue ("tppf1", TPPF1);
          cfg.lookupValue ("tppf2", TPPF2);
          cfg.lookupValue ("tppf3", TPPF3);
          cfg.lookupValue ("tppf4", TPPF4);
        } catch (SettingNotFoundException &nfex) {
            cerr << "Configuration not found!" << endl;
        }
    }
}
