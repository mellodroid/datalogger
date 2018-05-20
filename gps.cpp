#include <cstdint>
#include <cmath>
#include <string>
#include <iostream>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/spirit/home/x3.hpp>

#include "gps.h"

namespace gps
{
    geoPosition::geoPosition() : latitude(0), longitude(0), altitude(0) { }
    
    efecPosition::efecPosition() : x(0), y(0), z(0) { }
    
    polarVector::polarVector() : r(0), theta(0) { }
    
    satellite::satellite() : prn(0), elevation(0), azimuth(0), snr(0) { }
    
    namespace ast
    {
        namespace x3 = boost::spirit::x3;
        namespace ascii = boost::spirit::x3::ascii;
        
        using x3::double_;
        using x3::int_;
        using x3::lit;
        using ascii::space;
        using x3::_attr;
        using x3::char_;
        
        namespace defs
        {
            posFixMode_table::posFixMode_table() {
                add ("1", gps::positionFixMode::FixNotAvailable)
                    ("2", gps::positionFixMode::Fix2D)
                    ("3", gps::positionFixMode::Fix3D)
                    ;
            }
            
            posFixType_table::posFixType_table() {
                add ("A", gps::positionFixType::Automatic)
                    ("M", gps::positionFixType::Manual)
                    ;
            }
            
            posMode_table::posMode_table() {
                add ("A", gps::positionMode::Autonomous)
                    ("D", gps::positionMode::Differential)
                    ("E", gps::positionMode::Estimated)
                    ("M", gps::positionMode::Manual)
                    ("N", gps::positionMode::DataNotValid)
                    ;
            }
            
            direction_table::direction_table() {
                add ("N",  1)
                    ("S", -1)
                    ("E",  1)
                    ("W", -1)
                    ;
            }
            
            status_table::status_table() {
                add ("A", gps::dataStatus::Valid)
                    ("V", gps::dataStatus::Invalid)
                    ;
            }
            
            quality_table::quality_table() {
                add ("0", gps::qualityIndicator::FixInvalid)
                    ("1", gps::qualityIndicator::SinglePoint)
                    ("2", gps::qualityIndicator::PseudorangeDifferential)
                    ("4", gps::qualityIndicator::FixedRTK)
                    ("5", gps::qualityIndicator::FloatRTK)
                    ("6", gps::qualityIndicator::DeadReckoning)
                    ("7", gps::qualityIndicator::ManualInput)
                    ("8", gps::qualityIndicator::Simulator)
                    ("9", gps::qualityIndicator::SBAS)
                    ;                        
            }
        }

        namespace convert
        {
            using namespace boost::posix_time;
            using namespace boost::gregorian;

            ptime value2date(int value)
            {
                date d(2000 + value % 100, (value % 10000) / 100, value / 10000);
                
                return ptime(d);
            }
            
            time_duration value2time(double value)
            {
                //Converts HHMMSS to time_duration
                return hours(static_cast<int>(value) / 10000) 
                     + minutes((static_cast<int>(value) % 10000) / 100) 
                     + seconds(static_cast<int>(value) % 100) 
                     + millisec(static_cast<int>(fmod(value, 1) * 1000));

            }
            
            double value2position(double value)
            {
                return static_cast<double>(static_cast<int>(value) / 100) 
                     + (fmod(value, 100) / 60.0);
            }
        }
        
        enum class msgType
        {
            GGA,
            GLL,
            GSA,
            GSV,
            RMC,
            VTG,            
            Invalid
        };
        
        enum class satType
        {
            GPS,
            GLONASS,
            COMBINED,
            Invalid
        };

        template <typename Iterator>
        bool parse_gpvtg(Iterator first, Iterator last, gps::nmeaData& nmea)
        {
            using namespace boost::posix_time;
            namespace convert = gps::ast::convert;

            auto assign_true = [&](auto& ctx){ nmea.trackTrue.theta = _attr(ctx); };
            auto assign_mag  = [&](auto& ctx){ nmea.trackMagnetic.theta = _attr(ctx); };
            auto assign_spdt = [&](auto& ctx){ nmea.trackTrue.r = _attr(ctx) * 1.852; };
            auto assign_spdm = [&](auto& ctx){ nmea.trackMagnetic.r = _attr(ctx); };
            auto assign_mode = [&](auto& ctx){ nmea.mode = _attr(ctx); };
            
            bool r = x3::phrase_parse(first, last,

                //  Begin grammar
                (
                    (lit("$GP") | lit("$GN") | lit("$GL")) >>
                    lit("VTG") >> ','
                    >> (double_[assign_true] | x3::attr(0) ) >> ',' >> lit('T') >> ','
                    >> (double_[assign_mag]  | x3::attr(0) ) >> ',' >> lit('M') >> ','
                    >> (double_[assign_spdt] | x3::attr(0) ) >> ',' >> lit('N') >> ','
                    >> (double_[assign_spdm] | x3::attr(0) ) >> ',' >> lit('K') >> ','
                    >> (gps::ast::defs::posMode_attr[assign_mode] | x3::attr(gps::positionMode::DataNotValid)[assign_mode]) >> '*'
                    >> (+(char_))
                )
                ,
                //  End grammar

                space);

            if (first != last) // fail if we did not get a full match
                return false;
            return r;
        }
        
        template <typename Iterator>
        bool parse_gpgll(Iterator first, Iterator last, gps::nmeaData& nmea)
        {
            using namespace boost::posix_time;
            namespace convert = gps::ast::convert;

            auto assign_time = [&](auto& ctx){ nmea.utcTime = convert::value2time(_attr(ctx)); };
            auto assign_lat  = [&](auto& ctx){ nmea.position.latitude   = convert::value2position(_attr(ctx)); };
            auto modify_lat  = [&](auto& ctx){ nmea.position.latitude  *= _attr(ctx); };
            auto assign_lng  = [&](auto& ctx){ nmea.position.longitude  = convert::value2position(_attr(ctx)); };
            auto modify_lng  = [&](auto& ctx){ nmea.position.longitude *= _attr(ctx); };
            auto assign_sts  = [&](auto& ctx){ nmea.status = _attr(ctx); };
            auto assign_mode = [&](auto& ctx){ nmea.mode = _attr(ctx); };
            
            bool r = x3::phrase_parse(first, last,

                //  Begin grammar
                (
                    (lit("$GP") | lit("$GN") | lit("$GL")) >>
                    lit("GLL") >> ','
                    >> (double_[assign_lat]                       | x3::attr(0.0)[assign_lat]) >> ','
                    >> (gps::ast::defs::dir_attr[modify_lat]      | x3::attr(0)) >> ','
                    >> (double_[assign_lng]                       | x3::attr(0.0)[assign_lng]) >> ','
                    >> (gps::ast::defs::dir_attr[modify_lng]      | x3::attr(0)) >> ','
                    >> (double_[assign_time]                      | x3::attr(0.0)[assign_time]) >> ','
                    >> (gps::ast::defs::status_attr[assign_sts]   | x3::attr(gps::dataStatus::Invalid)[assign_sts]) >> ','
                    >> (gps::ast::defs::posMode_attr[assign_mode] | x3::attr(gps::positionMode::DataNotValid)[assign_mode]) >> '*'
                    >> (+(char_))
                )
                ,
                //  End grammar

                space);

            if (first != last) // fail if we did not get a full match
                return false;
            return r;
        }
        
        template <typename Iterator>
        bool parse_gpgga(Iterator first, Iterator last, gps::nmeaData& nmea)
        {
            using namespace boost::posix_time;
            namespace convert = gps::ast::convert;

            auto assign_time = [&](auto& ctx){ nmea.utcTime = convert::value2time(_attr(ctx)); };
            auto assign_lat  = [&](auto& ctx){ nmea.position.latitude   = convert::value2position(_attr(ctx)); };
            auto modify_lat  = [&](auto& ctx){ nmea.position.latitude  *= _attr(ctx); };
            auto assign_lng  = [&](auto& ctx){ nmea.position.longitude  = convert::value2position(_attr(ctx)); };
            auto modify_lng  = [&](auto& ctx){ nmea.position.longitude *= _attr(ctx); };
            auto assign_qual = [&](auto& ctx){ nmea.quality = _attr(ctx); };
            auto assign_sats = [&](auto& ctx){ nmea.satellitesInUseCount = _attr(ctx); };
            auto assign_hdop = [&](auto& ctx){ nmea.hdop = _attr(ctx); };
            auto assign_alt  = [&](auto& ctx){ nmea.position.altitude = _attr(ctx); };
            auto assign_und  = [&](auto& ctx){ nmea.undulation = _attr(ctx); };
            auto assign_age  = [&](auto& ctx){ nmea.ageOfCorrectionData = _attr(ctx); };
            auto assign_sid  = [&](auto& ctx){ nmea.differentialBaseStationID = _attr(ctx); };
            
            bool r = x3::phrase_parse(first, last,

                //  Begin grammar
                (
                    (lit("$GP") | lit("$GN") | lit("$GL")) >>
                    lit("GGA") >> ','
                    >> (double_[assign_time]                 | x3::attr(0.0)[assign_time]) >> ','
                    >> (double_[assign_lat]                  | x3::attr(0.0)[assign_lat]) >> ','
                    >> (gps::ast::defs::dir_attr[modify_lat] | x3::attr(0)) >> ','
                    >> (double_[assign_lng]                  | x3::attr(0.0)[assign_lng]) >> ','
                    >> (gps::ast::defs::dir_attr[modify_lng] | x3::attr(0)) >> ','
                    >> (gps::ast::defs::quality_attr[assign_qual] | x3::attr(gps::qualityIndicator::FixInvalid)[assign_qual]) >> ','
                    >> (int_[assign_sats]                    | x3::attr(0)) >> ','
                    >> (double_[assign_hdop]                 | x3::attr(0)) >> ','
                    >> (double_[assign_alt]                  | x3::attr(0)) >> ',' >> lit('M') >> ','
                    >> (double_[assign_und]                  | x3::attr(0)) >> ',' >> lit('M') >> ','
                    >> (int_[assign_age]                     | x3::attr(0)) >> ','
                    >> (int_[assign_sid]                     | x3::attr(0)) >> '*'
                    >> (+(char_))
                )
                ,
                //  End grammar

                space);

            if (first != last) // fail if we did not get a full match
                return false;
            return r;
        }
        
        template <typename Iterator>
        bool parse_gpgsa(Iterator first, Iterator last, gps::nmeaData& nmea)
        {            
            nmea.satellitesInUse.clear();
            
            auto get_fixtype = [&](auto& ctx){ nmea.fixType = _attr(ctx); };
            auto get_fixmode = [&](auto& ctx){ nmea.fixMode = _attr(ctx); };
            auto get_sat     = [&](auto& ctx){ nmea.satellitesInUse.push_back(_attr(ctx)); };
            auto get_pdop    = [&](auto& ctx){ nmea.pdop    = _attr(ctx); };
            auto get_hdop    = [&](auto& ctx){ nmea.hdop    = _attr(ctx); };
            auto get_vdop    = [&](auto& ctx){ nmea.vdop    = _attr(ctx); };
                    
            bool r = x3::phrase_parse(first, last,

                //  Begin grammar
                (
                    (lit("$GP") | lit("$GN") | lit("$GL")) >>
                    lit("GSA") >> ','
                    >> (gps::ast::defs::posFixType_attr[get_fixtype] | x3::attr(gps::positionFixType::Automatic)[get_fixtype]) >> ','
                    >> (gps::ast::defs::posFixMode_attr[get_fixmode] | x3::attr(gps::positionFixMode::FixNotAvailable)[get_fixmode]) >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (int_[get_sat]                                | x3::attr(0))  >> ','
                    >> (double_[get_pdop]                            | x3::attr(0)[get_pdop]) >> ','
                    >> (double_[get_hdop]                            | x3::attr(0)[get_hdop]) >> ','
                    >> (double_[get_vdop]                            | x3::attr(0)[get_vdop]) >> '*'
                    >> (+(char_))
                )
                ,
                //  End grammar

                space);
            
            if (first != last) // fail if we did not get a full match
                return false;
            return r;
        }
        
        template <typename Iterator>
        bool parse_gpgsv(Iterator first, Iterator last, gps::nmeaData& nmea)
        {
            uint8_t msgIdx = 1;
            
            gps::satellite sat;
            
            auto get_idx      = [&](auto& ctx){ msgIdx  = _attr(ctx); if(msgIdx == 1) { nmea.satellitesInView.clear(); } };
            auto get_count    = [&](auto& ctx){ nmea.satellitesInViewCount = _attr(ctx); };
            auto get_sat_prn  = [&](auto& ctx){ sat.prn       = _attr(ctx); };
            auto get_sat_el   = [&](auto& ctx){ sat.elevation = _attr(ctx); };
            auto get_sat_az   = [&](auto& ctx){ sat.azimuth   = _attr(ctx); };        
            auto get_sat_snr  = [&](auto& ctx){ sat.snr       = _attr(ctx); nmea.satellitesInView.push_back(sat); };
                    
            bool r = x3::phrase_parse(first, last,

                //  Begin grammar
                (
                    (lit("$GP") | lit("$GN") | lit("$GL")) >>
                    lit("GSV") >> ','
                    >> (int_              | x3::attr(0)) >> ','
                    >> (int_[get_idx]     | x3::attr(1)[get_idx]) >> ','
                    >> (int_[get_count]   | x3::attr(0)[get_count]) >> *(','
                    >> (int_[get_sat_prn] | x3::attr(0)[get_sat_prn]) >> ','
                    >> (int_[get_sat_el]  | x3::attr(0)[get_sat_el]) >> ','
                    >> (int_[get_sat_az]  | x3::attr(0)[get_sat_az]) >> ','
                    >> (int_[get_sat_snr] | x3::attr(0)[get_sat_snr])) >> '*'
                    >> (+(char_))
                )
                ,
                //  End grammar

                space);
            
            if (first != last) // fail if we did not get a full match
                return false;
            return r;
        }

        template <typename Iterator>
        bool parse_gprmc(Iterator first, Iterator last, gps::nmeaData& nmea)
        {
            using namespace boost::posix_time;
            namespace convert = gps::ast::convert;
            
            time_duration duration;
            
            double magVar;

            auto assign_date = [&](auto& ctx){ nmea.date = convert::value2date(_attr(ctx)); };
            auto assign_time = [&](auto& ctx){ nmea.utcTime = convert::value2time(_attr(ctx)); };
            auto assign_lat  = [&](auto& ctx){ nmea.position.latitude   = convert::value2position(_attr(ctx)); };
            auto modify_lat  = [&](auto& ctx){ nmea.position.latitude  *= _attr(ctx); };
            auto assign_lng  = [&](auto& ctx){ nmea.position.longitude  = convert::value2position(_attr(ctx)); };
            auto modify_lng  = [&](auto& ctx){ nmea.position.longitude *= _attr(ctx); };
            auto assign_sts  = [&](auto& ctx){ nmea.status = _attr(ctx); };
            auto assign_vel  = [&](auto& ctx){ nmea.trackTrue.r     = _attr(ctx) * 1.852; nmea.trackMagnetic.r = _attr(ctx) * 1.852; };
            auto modify_vel  = [&](auto& ctx){ nmea.trackTrue.theta = _attr(ctx); };
            auto assign_mag  = [&](auto& ctx){ magVar  = _attr(ctx); };
            auto modify_mag  = [&](auto& ctx){ magVar *= _attr(ctx); nmea.trackMagnetic.theta = nmea.trackTrue.theta - magVar; };
            auto assign_mode = [&](auto& ctx){ nmea.mode = _attr(ctx); };
//            auto assign_sum  = [&](auto& ctx){ rmc.checksum = std::stoul(_attr(ctx), nullptr, 16); };
            
            bool r = x3::phrase_parse(first, last,

                //  Begin grammar
                (
                    (lit("$GP") | lit("$GN") | lit("$GL")) >>
                    lit("RMC") >> ','
                    >> (double_[assign_time]       | x3::attr(0.0)[assign_time]) >> ','
                    >> (gps::ast::defs::status_attr[assign_sts]     | x3::attr(gps::dataStatus::Invalid)[assign_sts]) >> ','
                    >> (double_[assign_lat]        | x3::attr(0.0)[assign_lat]) >> ','
                    >> (gps::ast::defs::dir_attr[modify_lat] | x3::attr(0)) >> ','
                    >> (double_[assign_lng]        | x3::attr(0.0)[assign_lng]) >> ','
                    >> (gps::ast::defs::dir_attr[modify_lng] | x3::attr(0)) >> ','
                    >> (double_[assign_vel]        | x3::attr(0.0)[assign_vel]) >> ','
                    >> (double_[modify_vel]        | x3::attr(0.0)) >> ','
                    >> (int_[assign_date]          | x3::attr(0)[assign_date]) >> ','
                    >> (double_[assign_mag]        | x3::attr(0.0)[assign_mag]) >> ','
                    >> (gps::ast::defs::dir_attr[modify_mag] | x3::attr(0.0)[modify_mag]) >> ','
                    >> (gps::ast::defs::posMode_attr[assign_mode]     | x3::attr(gps::positionMode::DataNotValid)[assign_mode]) >> '*'
                    >> (+(char_))
                )
                ,
                //  End grammar

                space);

            if (first != last) // fail if we did not get a full match
                return false;
            return r;
        }
        
        bool parseNmeaMessage(std::string str, gps::nmeaData& nmea)
        {
            satType sat = satType::Invalid;
            msgType msg = msgType::Invalid;
            
            // Determine Satellite Type
            if (str.compare(0, 3, "$GP") == 0)
                sat = satType::GPS;
            else if (str.compare(0, 3, "$GL") == 0)
                sat = satType::GLONASS;
            else if (str.compare(0, 3, "$GN") == 0)
                sat = satType::COMBINED;
                
            // Determine Message Type
            if (str.compare(3, 3, "GGA") == 0)
                msg = msgType::GGA;
            else if (str.compare(3, 3, "GLL") == 0)
                msg = msgType::GLL;
            else if (str.compare(3, 3, "GSA") == 0)
                msg = msgType::GSA;
            else if (str.compare(3, 3, "GSV") == 0)
                msg = msgType::GSV;
            else if (str.compare(3, 3, "RMC") == 0)
                msg = msgType::RMC;
            else if (str.compare(3, 3, "VTG") == 0)
                msg = msgType::VTG;
                
            // Don't process if msg is GLONASS, or just the wrong thing...
            if (sat == satType::Invalid || sat == satType::GLONASS || msg == msgType::Invalid)
                return false;

            switch (msg)
            {
                case msgType::GGA:
                    gps::ast::parse_gpgga(str.begin(), str.end(), nmea);
                    break;
                case msgType::GLL:
                    gps::ast::parse_gpgll(str.begin(), str.end(), nmea);
                    break;
                case msgType::GSA:
                    gps::ast::parse_gpgsa(str.begin(), str.end(), nmea);
                    break;
                case msgType::GSV:
                    gps::ast::parse_gpgsv(str.begin(), str.end(), nmea);
                    break;
                case msgType::RMC:
                    gps::ast::parse_gprmc(str.begin(), str.end(), nmea);
                    break;
                case msgType::VTG:
                    gps::ast::parse_gpvtg(str.begin(), str.end(), nmea);
                    break;
            }
        }
    }
    
}
