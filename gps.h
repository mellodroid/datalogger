#ifndef _GPS_HEADER_
#define _GPS_HEADER_

/****************************************************************************
 * GPS provides interfaces for storing, timestamping, and decoding NMEA0183
 * compatible messages.
 * 
 * Beyond NMEA messaging, this library will also convert between ECEF and 
 * WGS84 geodetic coordinates for the purposes of computing changes in 
 * position based on track velocity.
 * 
 * Author: Steve Kik
 * *************************************************************************/
 
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/spirit/home/x3.hpp>
 
#include <cstdint> 
#include <string>
 
namespace gps 
{
    struct geoPosition
    {
        double latitude;
        double longitude;
        double altitude;
        
        geoPosition();
    };
    
    struct efecPosition
    {
        double x;
        double y;
        double z;
        
        efecPosition();
    };
    
    struct polarVector
    {
        double r;
        double theta;
        
        polarVector();
    };
    
    struct satellite
    {
        uint8_t  prn;
        uint16_t elevation;
        uint16_t azimuth;
        uint8_t  snr;
        
        satellite();
    };
    
    enum class dataStatus
    {
        Invalid,
        Valid
    };
    
    enum class usageStatus
    {
        NotInUse,
        InUse,
        Unknown
    };
    
    enum class positionMode
    {
        DataNotValid,
        Autonomous,
        Differential,
        Estimated,
        Manual
    };
    
    enum class qualityIndicator
    {
        FixInvalid,
        SinglePoint,
        PseudorangeDifferential,
        FixedRTK,
        FloatRTK,
        DeadReckoning,
        ManualInput,
        Simulator,
        SBAS
    };
    
    enum class positionFixType
    {
        Automatic,
        Manual
    };
    
    enum class positionFixMode
    {
        FixNotAvailable,
        Fix2D,
        Fix3D
    };
    
    struct nmeaData
    {
                                       // Fields get filled from these messages:
        geoPosition            position;                  // G*GLL, G*GGA, G*RMC
        polarVector            trackTrue;                 // G*VTG, G*RMC
        polarVector            trackMagnetic;             // G*VTG, G*RMC
        boost::posix_time::time_duration    utcTime;      // G*GLL, G*GGA, G*RMC
        boost::posix_time::ptime            date;         // G*RMC
        float                  pdop;                      // G*GSA
        float                  hdop;                      // G*GSA, G*GGA
        float                  vdop;                      // G*GSA
        double                 undulation;                // G*GGA
        std::vector<uint8_t>   satellitesInUse;           // G*GSA, G*GGA
        uint8_t                satellitesInUseCount;      // G*GGA
        std::vector<satellite> satellitesInView;          // G*GSV
        uint8_t                satellitesInViewCount;     // G*GSV
        dataStatus             status;                    // G*GLL, G*RMC
        positionMode           mode;                      // G*GLL, G*RMC, G*VTG
        positionFixType        fixType;                   // G*GSA
        positionFixMode        fixMode;                   // G*GSA
        uint8_t                differentialBaseStationID; // G*GGA
        uint8_t                ageOfCorrectionData;       // G*GGA
        qualityIndicator       quality;                   // G*GGA
    };
    
    namespace ast
    {
        namespace x3 = boost::spirit::x3;
        
        namespace defs {
                
            struct posFixMode_table : x3::symbols<gps::positionFixMode> {
                posFixMode_table(); 
            } const posFixMode_attr;
            
            struct posFixType_table : x3::symbols<gps::positionFixType> {
                posFixType_table();
            } const posFixType_attr;
            
            struct posMode_table : x3::symbols<gps::positionMode> {
                posMode_table();
            } const posMode_attr;
            
            struct direction_table : x3::symbols<double> {
                direction_table();
            } const dir_attr;
            
            struct status_table : x3::symbols<gps::dataStatus> {
                status_table();
            } const status_attr;
            
            struct quality_table : x3::symbols<gps::qualityIndicator> {
                quality_table();
            } const quality_attr;
            
        }
        
        namespace convert {
            using namespace boost::posix_time;
            using namespace boost::gregorian;
            
            time_duration value2time(double value);
            
            ptime value2date(int value);
            
            double value2position(double value);
        }
        
        bool parseNmeaMessage(std::string str, gps::nmeaData& nmea);
    }
}

#endif