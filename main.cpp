#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

#include <ncurses.h>

#include <cstdint>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "gps.h"
#include "serial.h"
#include "MMA8451.h"

////////////////////////////////////////////////////////////////////////////
//  Main program
////////////////////////////////////////////////////////////////////////////
int
main()
{

    initscr();

    WINDOW *win  = newwin(11, 55, 1, 1);
    wborder(win, 0, 0, 0, 0, 0, 0, 0, 0);
    WINDOW *win2 = newwin(5, 55, 14, 1);
    wborder(win2, 0, 0, 0, 0, 0, 0, 0, 0);
    
    char strbuf[100];
    std::string str;

    gps::nmeaData nmeaData;
    
    serial::SerialPort serport;
    
    mma8451::Mma8451 accelerometer;
    
    serport.openPort("/dev/serial0");
    
    accelerometer.enableLowNoiseMode(true);
    accelerometer.setDataRate(mma8451::DataRate::RATE_100HZ);
    accelerometer.setFullScale(mma8451::FullScale::FULL_SCALE_2G);
    accelerometer.setHighPassOutput(false);
    accelerometer.setActivePowerMode(mma8451::PowerMode::HIGH_RESOLUTION);

    while(true) {
        str = serport.readPort();
        
        std::vector<float> accels = accelerometer.getAccelerations();
       
	if (str.size() < 6)
	   continue;
	else if (gps::ast::parseNmeaMessage(str, nmeaData))
        {
	    std::ostringstream oss;
	    oss << nmeaData.date + nmeaData.utcTime;
	    wmove(win, 1, 1);
	    wprintw(win, "DateTime:");
	    wmove(win, 1, 15);
	    wprintw(win, oss.str().c_str());
	    wmove(win, 3, 1);
	    wprintw(win, "Position:");
	    wmove(win, 3, 14);
	    wprintw(win, "% 03.6f, % 03.6f, % 04.2fm", nmeaData.position.latitude, nmeaData.position.longitude, nmeaData.position.altitude);
	    wmove(win, 5, 1);
	    wprintw(win, "Velocity:");
	    wmove(win, 5, 14);
	    wprintw(win, "% 04.2fkph, % 01.2fdeg", nmeaData.trackTrue.r, nmeaData.trackTrue.theta);
	    wmove(win, 7, 1);
	    wprintw(win, "PDOP:");
	    wmove(win, 7, 14);
	    wprintw(win, "% 01.2f                                       ", nmeaData.pdop);
	    wmove(win, 9, 1);
	    wprintw(win, "Satellites:");
	    wmove(win, 9, 14);
	    wprintw(win, "% 02d viewable, % 02d in use", nmeaData.satellitesInViewCount, nmeaData.satellitesInUseCount);
        
        wmove(win2, 2, 1);
        wprintw(win2, "Acceleration:");
        wmove(win2, 2, 14);
        std::sprintf(strbuf, "% 02.2f, % 02.2f, % 02.2f", accels[0], accels[1], accels[2]);
        wprintw(win2, strbuf);
        
	    wrefresh(win);        
        wrefresh(win2);

//            std::cout << "DateTime: " << nmeaData.date + nmeaData.utcTime << std::endl;
//            std::cout << "Position: " << nmeaData.position.latitude << ", " << nmeaData.position.longitude << std::endl;
//            std::cout << "Velocity: " << nmeaData.trackTrue.r << " kph, " << nmeaData.trackTrue.theta << " deg" << std::endl;
//            std::cout << "PDOP:     " << nmeaData.pdop<< std::endl;
        }
    }        
    
    serport.closePort();
    

//    while (getline(std::cin, str))
//    {
//        if (str.empty() || str[0] == 'q' || str[0] == 'Q')
//            break;
//
//        
//        if (gps::ast::parseNmeaMessage(str, nmeaData))
//        {
//            std::cout << "DateTime: " << nmeaData.date + nmeaData.utcTime << std::endl;
//            std::cout << "Position: " << nmeaData.position.latitude << ", " << nmeaData.position.longitude << std::endl;
//            std::cout << "Velocity: " << nmeaData.trackTrue.r << " kph, " << nmeaData.trackTrue.theta << " deg" << std::endl;
//            std::cout << "Active:   " << (nmeaData.status == gps::dataStatus::Valid) << std::endl;
//            
//            for (auto iter = nmeaData.satellitesInView.begin(); iter != nmeaData.satellitesInView.end(); ++iter) {
//                std::cout << "PRN: " << (int)iter->prn << ",\tSNR: " << (int)iter->snr << ",\tAz/El: (" << (int)iter->azimuth << ", " << (int)iter->elevation << ")" << std::endl;
//            }
//            
//            for (auto iter = nmeaData.satellitesInUse.begin(); iter != nmeaData.satellitesInUse.end(); ++iter) {
//                std::cout << "PRN: " << (int)*iter << std::endl;
//            }
//        }
//        else
//        {
//            std::cout << "-------------------------\n";
//            std::cout << "Parsing failed\n";
//            std::cout << "-------------------------\n";
//        }
//    }
//
//    std::cout << "Bye... :-) \n\n";
    endwin();
    return 0;
}
