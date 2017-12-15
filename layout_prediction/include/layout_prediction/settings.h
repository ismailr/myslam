#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include <iostream>
#include <libconfig.h++>

using namespace std;
using namespace libconfig;

namespace myslam {
    namespace settings {

        int SIMULATION;
        int SLAM_ONLINE;

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
            } catch (SettingNotFoundException &nfex) {
                cerr << "Configuration not found!" << endl;
            }
        }
    }
}
#endif
