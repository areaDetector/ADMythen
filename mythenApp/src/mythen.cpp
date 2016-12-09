/* mythen.cpp
 *
 * This is a driver for Dextris Mythen Detector.
 *
 * Based on the slsDetector driver by Xiaoqiang Wang (PSI) - June 8, 2012
 * Based on the asynPort driver from LNLS
 *
 * Author: J. Sullivan
 *         ANL - APS/XSD/BCDA
 *
 * Created:  June 17 2015
 *
 * Modified:
 *            2015-07-14  M. Moore ANL - APS/XSD/DET: Upated to work with other firmwares
 *            2015-07-14  M. Moore ANL - APS/XSD/DET: Added ReadMode to allow Reading of corrected data from Detector
 *            2015-07-15  M. Moore ANL - APS/XSD/DET: Modified acquire sequence to handle trigger time outs
 *            2015-08-05  M. Moore ANL - APS/XSD/DET: Changed how readout timeouts are handled to be timeout+acquiretime
 *                                                    Handles ImageMode correctly, so that if you have single acquire it only acquires one image
 *
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEndian.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"
#include "NDPluginDriver.h"

#include <epicsExport.h>

#include <stdexcept>
#include <vector>
#include <climits>

#define MAX_FILENAME_LEN 256
#define MAX_DIMS      1280
#define MAX_COMMAND_LEN 128
#define MAX_NMODULES 2
#define M1K_TIMEOUT 5.0
#define MAX_FRAMES 500
#define MAX_TRIGGER_TIMEOUT_COUNT 50
#define FIRMWARE_VERSION_LEN 7

static const char *driverName = "mythen";

#define SDSettingString          "SD_SETTING"
#define SDDelayTimeString        "SD_DELAY_TIME"
#define SDThresholdString        "SD_THRESHOLD"
#define SDEnergyString           "SD_ENERGY"
#define SDUseFlatFieldString     "SD_USE_FLATFIELD"
#define SDUseCountRateString     "SD_USE_COUNTRATE"
#define SDTauString              "SD_TAU"
#define SDUseBadChanIntrplString "SD_USE_BADCHANNEL_INTRPL"
#define SDBitDepthString         "SD_BIT_DEPTH"
#define SDUseGatesString         "SD_USE_GATES"
#define SDNumGatesString         "SD_NUM_GATES"
#define SDNumFramesString        "SD_NUM_FRAMES"
#define SDTriggerString          "SD_TRIGGER"
#define SDResetString            "SD_RESET"
#define SDNModulesString         "SD_NMODULES"
#define SDFirmwareVersionString  "SD_FIRMWARE_VERSION"  /* asynOctet    ro */
#define SDReadModeString         "SD_READ_MODE"

namespace mythen {

class MythenConnectionError : public std::runtime_error {
public:
    MythenConnectionError(const char* error)
        : std::runtime_error(error)
    { };
};

/** 
 * Casts a byte array to the specific type and also swaps bytes if the host uses
 * big endian format.
 *
 * This implementation seems to be on the slower side but simplifies the interface,
 * consider adding specializations.
 */
template <class T>
T byteCast(char* str)
{
    T value; 

    if (EPICS_BYTE_ORDER == EPICS_ENDIAN_BIG) {
        char* pVal = reinterpret_cast<char*>(&value);
        size_t i;
        for (i = 0; i < sizeof(T); ++i) {
            pVal[sizeof(T)-1-i] = str[i];
        }
    } else {
        value = *reinterpret_cast<T*>(str);
    }
    return value;
}

/**
 * Mythen firmware version utility class.
 */
class FirmwareVersion {
public:
    /* These are capitalized due to macro definition in glibc:
     * https://bugzilla.redhat.com/show_bug.cgi?id=130601
     */
    const int MAJOR;
    const int MINOR;
    const int RELEASE; 
    const std::string version_str;

    /**
     * Constructor.
     *
     * The version numbers are parsed from 7 byte long string
     * of type "M3.0.0". 
     *
     * \param version Version string.
     */
    FirmwareVersion(std::string version)
        : MAJOR(version[1]%48),
          MINOR(version[3]%48),
          RELEASE(version[5]%48),
          version_str(version)
    { }

    /**
     * Default constructor.
     */
    FirmwareVersion()
        : MAJOR(0),
          MINOR(0),
          RELEASE(0)
    { }

    FirmwareVersion operator=(const FirmwareVersion& fw)
    {
        return FirmwareVersion(fw.version_str);
    }

};

/**
 * Driver for Dectris Mythen detector.
 *
 * This driver uses the socket interface over TCP/UDP.
 */
class mythen : public ADDriver {
    public:
        mythen(const char *portName, const char *IPPortName,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

        /* These are the methods that we override from ADDriver */
        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        asynStatus writeOctet(asynUser *pasynUser, const char *value,
                size_t nChars, size_t *nActual); // TODO not really used
        void report(FILE *fp, int details);

        /** Used to start the internal thread for acquisition */
        friend void acquisitionTaskC(void *drvPvt);

    protected:
        int SDSetting;
#define FIRST_SD_PARAM SDSetting
        int SDDelayTime;
        int SDThreshold;
        int SDEnergy;
        int SDUseFlatField;
        int SDUseCountRate;
        int SDUseBadChanIntrpl;
        int SDBitDepth;
        int SDUseGates;
        int SDNumGates;
        int SDNumFrames;
        int SDTrigger;
        int SDReset;
        int SDTau;
        int SDFirmwareVersion;
        int SDReadMode;
        int SDNModules;
#define LAST_SD_PARAM SDNModules

    private:
        enum SettingPreset {
            SettingPreset_Cu = 0,
            SettingPreset_Mo = 1,
            SettingPreset_Ag = 2, //Not supported on firmware verions less than 3
            SettingPreset_Cr = 3
        };

        enum TriggerMode {
            TriggerMode_None = 0,
            TriggerMode_Single = 1,
            TriggerMode_Continuous = 2
        };

    private:                                       
        /* These are the methods we implement from Mythen */
        void acquisitionTask();
        asynStatus setAcquire(epicsInt32 value);
        asynStatus setFCorrection(epicsInt32 value);
        asynStatus setRCorrection(epicsInt32 value);
        asynStatus setExposureTime(epicsFloat64 value);
        asynStatus setDelayAfterTrigger(epicsFloat64 value);
        asynStatus setBitDepth(epicsInt32 value);
        asynStatus setBadChanIntrpl(epicsInt32 value);
        asynStatus setUseGates(epicsInt32 value);
        asynStatus setNumGates(epicsInt32 value);
        asynStatus setKthresh(epicsFloat64 value);
        asynStatus setEnergy(epicsFloat64 value);
        asynStatus setTau(epicsFloat64 value);
        asynStatus setTrigger(epicsInt32 value);
        asynStatus loadSettings(epicsInt32 value);
        asynStatus setReset();
        asynStatus getSettings();
        epicsInt32 getStatus();
        asynStatus getFirmware();
        void decodeRawReadout(int nmods, int nbits, epicsUInt32 *data, epicsUInt32 *result);
        asynStatus sendCommand(const char* format, ...);
        template <class T> T writeReadNumeric(const char* inString);
        template <int N> std::string writeReadOctet(const char* inString);
        epicsInt32 dataCallback(epicsUInt32 *pData);

        epicsEventId startEventId_;
        asynUser *pasynUserMeter_;
        epicsInt32 acquiring_;
        epicsInt32 chanperline_;
        epicsInt32 nbits_;
        epicsInt32 nmodules_;
        epicsInt32 readmode_;
        FirmwareVersion fwVersion_;
};

#define NUM_SD_PARAMS (&LAST_SD_PARAM - &FIRST_SD_PARAM + 1)


/** Sends a command to the detector and reads the response.**/
asynStatus mythen::sendCommand(const char * format, ...)
{
    va_list args;
    const char *functionName = "sendCommand";
    char outString[MAX_COMMAND_LEN];

    va_start(args, format);
    int str_chars = sprintf(outString, format, args);
    va_end(args);
    if (str_chars < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error, formatting of command failed %s\n",
                driverName, functionName, format);
        return asynError;
    }

    try {
        epicsInt32 result = writeReadNumeric<epicsInt32>(outString);
        if (result != 0) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error, command %s execution failed: expected 0, "
                    "received %d.\n",
                    driverName, functionName, outString, result);
            return asynError;
        }
        return asynSuccess;
    } catch (const MythenConnectionError& e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error, Connection failed while issuing command %s.\n",
                driverName, functionName, outString);
        return asynError;
    }
}

template <class T>
T mythen::writeReadNumeric(const char * outString)
{
    size_t nread, nwrite;
    int eomReason;
    char inString[sizeof(T)];

    asynStatus status = pasynOctetSyncIO->writeRead(pasynUserMeter_, outString,
            strlen(outString), inString, sizeof(inString), M1K_TIMEOUT,
            &nwrite, &nread, &eomReason);

    if (status != asynSuccess) {
        throw MythenConnectionError(outString);
    }

    return byteCast<T>(inString);
}

template <int N>
std::string mythen::writeReadOctet(const char * outString)
{
    size_t nread, nwrite;
    int eomReason;
    char inString[N];

    asynStatus status = pasynOctetSyncIO->writeRead(pasynUserMeter_,
            outString, strlen(outString), inString, 
            N, M1K_TIMEOUT, &nwrite, &nread, &eomReason);

    if (status != asynSuccess) {
        throw MythenConnectionError(outString);
    }

    return std::string(inString);
}

/** Starts and stops the acquisition. **/
asynStatus mythen::setAcquire(epicsInt32 value)
{
    asynStatus status = asynSuccess;
    static const char *functionName = "setAcquire";
    if (value == 0) {
        status = sendCommand("-stop");
        setIntegerParam(ADStatus, getStatus());
        acquiring_ = 0;
    } else if (not acquiring_) {
        status = sendCommand("-start");
        // Notify the read thread that acquisition has started
        epicsEventSignal(startEventId_);
        acquiring_ = 1;
    } else {
        // We are already acquiring.
        status = asynSuccess;
    }

    if(status != asynSuccess) {
        acquiring_ = 0;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error setting acquistion!\n",
                driverName, functionName);
    }
    return status;
}

/** Sets the Trigger Mode */
asynStatus mythen::setTrigger(epicsInt32 value)
{
    switch (value) {
        case TriggerMode_None:
            // Clearing trigen clears conttrig also
            return sendCommand("-trigen 0");
        case TriggerMode_Single:
            return sendCommand("-trigen 1");
        case TriggerMode_Continuous:
            return sendCommand("-conttrigen 1");
        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "Unknown trigger mode %d \n.", value);
            return asynError;
    }

}


/** Sets the dead time constant for the rate correction**/
asynStatus mythen::setTau(epicsFloat64 value)
{
    if(value == -1 || value > 0) {
        return sendCommand("-tau %f", value);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "error check if tau -1 or >0 (value = %f)\n", value);
        return asynError;
    }
}


/** Sets the energy threshold for the module **/
asynStatus mythen::setKthresh(epicsFloat64 value)
{
    epicsInt32 i;
    int status = asynSuccess;

    if (value < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Energy threshold must be positive."
                " (setKthresh called with %f)\n", value);
        return asynError;
    }

    for(i=0;i<nmodules_;++i) {
        status |= sendCommand("-module %d", i);
        status |= sendCommand("-kthresh %f", value);
    }

    if(status == asynSuccess) {
        setDoubleParam(SDThreshold,value);
        callParamCallbacks();
    } else {
        setDoubleParam(SDThreshold,0);
        callParamCallbacks();

        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Unable to set kthresh to %f.\n", value);
        return asynError;
    }

    return asynSuccess;
}


/** Sets the energy threshold for the module **/
asynStatus mythen::setEnergy(epicsFloat64 value)
{
    epicsInt32 i;
    int status = asynSuccess;

    if (value < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "X-ray energy must be positive."
                " (setEnergy called with %f)\n", value);
        return asynError;
    }

    if (fwVersion_.MAJOR >= 3) {
        for(i=0;i<nmodules_;++i) {
            status |= sendCommand("-module %d", i);
            status |= sendCommand("-energy %f", value);
        }
        if(status != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "Unable to set X-ray energy (value = %f)\n", value);
            return asynError;
        }
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "X-ray energy cannot be set on this firmware.\n");
        return asynError;
    }

    return asynSuccess;
}

/** Sets the exposure time of one frame. (units of 100ns) **/
asynStatus mythen::setExposureTime(epicsFloat64 value)
{
    if (value < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Exposure time cannot be negative (reqested %f).\n", value);
        return asynError;
    }
    if (value > LLONG_MAX * 1E-7) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Reqested exposure time (%f) too long. "
                "Maximum value is %f.\n",
                value, LLONG_MAX*1E-7);
        return asynOverflow;
    }

    long long hns = (long long)(value * (1E+7));
    return sendCommand("-time %lld", hns);
}

/** Sets the delay time after trigger. (units of 100ns) **/
asynStatus mythen::setDelayAfterTrigger(epicsFloat64 value)
{
    if (value < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Trigger delay time cannot be negative (reqested %f).\n", value);
        return asynError;
    }
    if (value > LLONG_MAX * 1E-7) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Requested trigger delay time (%f) too long. "
                "Maximum value is %f.\n", value, LLONG_MAX * 1E-7);
        return asynOverflow;
    }

    long long hns = (long long)(value * (1E+7));
    return sendCommand("-delafter %d", hns);
}

/** Enables or disables the flatfield correction.
  After initialisation the flatfield correction is enabled. **/
asynStatus mythen::setFCorrection(epicsInt32 value)
{
    return sendCommand("-flatfieldcorrection %d", value);
}

/** Enables or disables the bad channel interpolation **/
asynStatus mythen::setBadChanIntrpl(epicsInt32 value)
{
    return sendCommand("-badchannelinterpolation %d", value);
}

/** Enables or disables the rate correction.
  After initialisation the rate correction is disabled. **/
asynStatus mythen::setRCorrection(epicsInt32 value)
{
    return sendCommand("-ratecorrection %d", value);
}

/** Enables or disables the gates.
  After initialisation the gates are disabled. **/
asynStatus mythen::setUseGates(epicsInt32 value)
{
    return sendCommand("-gateen %d", value);
}

/** Number of gates. **/
asynStatus mythen::setNumGates(epicsInt32 value)
{
    return sendCommand("-gates %d", value);
}

/** Get Firmware Version **/
asynStatus mythen::getFirmware()
{
    try {
        std::string fw_result =
            writeReadOctet<FIRMWARE_VERSION_LEN>("-get version");
        fwVersion_ = FirmwareVersion(fw_result);
        return asynSuccess;
    } catch (const MythenConnectionError& e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Unable to read firmware version.\n",
                driverName);
        return asynError;
    }
}

/** Get Acquition Status */
epicsInt32 mythen::getStatus()
{
    epicsInt32 detStatus;

    try {
        epicsInt32 aux = writeReadNumeric<epicsInt32>("-get status");
        int m_status = aux & 1;       // Acquire running status when non-zero
        int t_status = aux & (1<<3);  // Waiting for trigger when non-zero
        int d_status = aux & (1<<16); // No Data Available when not zero
        int triggerWaitCnt=0;
        double triggerWait;

        if (m_status || not d_status) {
            detStatus = ADStatusAcquire;

            triggerWaitCnt=0;
            // Waits for Trigger for increasing amount of time for a total of 
            // almost 1 minute
            while ((t_status ) && (triggerWaitCnt<MAX_TRIGGER_TIMEOUT_COUNT)) {
                triggerWait = 0.0001*pow(10.0,((double)(triggerWaitCnt/10)+1));
                epicsThreadSleep(triggerWait);
                aux = writeReadNumeric<epicsInt32>("-get status");
                t_status = aux & (1<<3);
                d_status = aux & (1<<16);
                triggerWaitCnt++;
            }

            if (not d_status) {
                detStatus = ADStatusReadout;
            } else if (triggerWaitCnt>=MAX_TRIGGER_TIMEOUT_COUNT) {
                detStatus = ADStatusError;
            }
        } else {
            detStatus = ADStatusIdle;
        }

        return detStatus;
    } catch (const MythenConnectionError& e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Connection error: Unable to retrieve status.\n",
                driverName);
        return ADStatusError;
    }
}

/**Enables or disables the flipping of the channel numbering. **/
asynStatus mythen::setBitDepth(epicsInt32 value)
{
    asynStatus status;
    epicsInt32 nbits;
    switch (value) {
        case 1:
            nbits = 16;
            break;
        case 2:
            nbits = 8;
            break;
        case 3:
            nbits = 4;
            break;
        case 0:
        default:
            nbits = 24;
            break;
    }

    status = sendCommand("-nbits %d", nbits);
    return status;
}

/**Loads predefined settings for the current module to measure
  some common x-ray radiation. The command loads the energy calibration,
  the bad channels file, the flatfield correction, and the trimbits
  for the corresponding settings, and sets the energy threshold to a
  suitable value.**/
asynStatus mythen::loadSettings(epicsInt32 value)
{
    int status = asynSuccess;
    epicsInt32 i = 0;

    for (i=0;i<nmodules_;++i) {
        status = sendCommand("-module %d", i);

        switch (value) {
            case SettingPreset_Cu:
                if (fwVersion_.MAJOR >= 3) {
                    status |= sendCommand("-settings Cu");
                } else {
                    status |= sendCommand("-settings StdCu");
                }
                break;

            case SettingPreset_Mo:
                if (fwVersion_.MAJOR >= 3) {
                    status |= sendCommand("-settings Mo");
                } else {
                    status |= sendCommand("-settings StdMo");
                }
                break;

            case SettingPreset_Ag:
                if (fwVersion_.MAJOR >= 3) {
                    status |= sendCommand("-settings Ag");
                } else {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s: Ag setting is not supported on this version"
                            "of firmware (%s)\n",
                            driverName, fwVersion_.version_str.c_str());
                    return asynError;
                }
                break;

            case SettingPreset_Cr:
                if (fwVersion_.MAJOR >= 3) {
                    status |= sendCommand("-settings Cr");
                } else {
                    status |= sendCommand("-settings HgCr");
                }
                break;

            default:
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s: Unknown setting %d, nothing was done.\n",
                    driverName, value);
                return asynError;
        }
    }

    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Unable to set the settings to %d.\n",
                driverName, value);
        return asynError;
    }

    setIntegerParam(SDSetting,value);

    callParamCallbacks();

    return asynSuccess;
}

/**Sets the detector back to default settings. This command takes
  about two seconds per module.**/
asynStatus mythen::setReset()
{
    asynStatus status=asynSuccess;
    epicsInt32 i=0;

    setIntegerParam(SDReset,1);
    for (i=0;i<nmodules_;++i) {
        status = sendCommand("-module %d", i);
        status = sendCommand("-reset");
    }
    setIntegerParam(SDReset,0);
    callParamCallbacks();
    return status;
}


/** Reads the values of all the modules parameters, sets them in the parameter library**/
asynStatus mythen::getSettings()
{
    static const char *functionName = "getSettings";

    if (acquiring_) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s%s: Cannot update settings while acquiring.\n",
                driverName, functionName);
        return asynError;
    }

    try {
        int aux = 0;
        epicsFloat32 faux = 0.;
        long long laux = 0;
        epicsFloat64 DetTime = 0.;

        aux = writeReadNumeric<epicsInt32>("-get flatfieldcorrection");
        if (aux!=0 && aux!=1) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s%s: Invalid value %d received for flat field correction.\n",
                    driverName, functionName, aux);
            return asynError;
        }
        setIntegerParam(SDUseFlatField, aux);


        aux = writeReadNumeric<epicsInt32>("-get badchannelinterpolation");
        if (aux!=0 && aux!=1) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s%s: Invalid value %d received for interpolation of "
                    "bad channels.\n",
                    driverName, functionName, aux);
            return asynError;
        }
        setIntegerParam(SDUseBadChanIntrpl, aux);

        aux = writeReadNumeric<epicsInt32>("-get ratecorrection");
        if (aux!=0 && aux!=1) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s%s: Invalid value %d received for rate correction.\n",
                    driverName, functionName, aux);
            return asynError;
        }
        setIntegerParam(SDUseCountRate, aux);

        aux = writeReadNumeric<epicsInt32>("-get nbits");
        if (aux < 0) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s%s: Invalid value %d received for nbits.\n",
                    driverName, functionName, aux);
            return asynError;
        }
        nbits_ = aux;
        chanperline_ = 32/aux;
        setIntegerParam(SDBitDepth, aux);

        aux = writeReadNumeric<epicsInt32>("-get time");
        if (aux >= 0) {
            DetTime = (aux * (1E-7));
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s%s: Invalid value %d received for exposure time.\n",
                    driverName, functionName, aux);
            return asynError;
        }
        setDoubleParam(ADAcquireTime, DetTime);


        aux = writeReadNumeric<epicsInt32>("-get frames");
        if (aux >= 0) {
            setIntegerParam(SDNumFrames, aux);
        }

        faux = writeReadNumeric<epicsFloat32>("-get tau");
        if (faux == -1 || faux > 0) {
            setDoubleParam(SDTau, faux);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s%s: Invalid value %d received for tau.\n",
                    driverName, functionName, aux);
            return asynError;
        }


        faux = writeReadNumeric<epicsFloat32>("-get kthresh");
        if (faux < 0) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s%s: Invalid value %f received for threshold.\n",
                    driverName, functionName, faux);
            return asynError;
        }
        else setDoubleParam(SDThreshold,faux);

        /* Newer versions of firmware provide additional commands. */
        if (fwVersion_.MAJOR >= 3) {
            faux = writeReadNumeric<epicsFloat32>("-get energy");
            if (faux < 0) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s%s: Invalid value %f received for energy.\n",
                        driverName, functionName, faux);
                return asynError;
            }
            else setDoubleParam(SDEnergy,faux);

            laux = writeReadNumeric<long long>("-get delafter");
            if (laux >= 0) {
                DetTime = (laux * (1E-7));
            }
            else {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s%s: Invalid value %lld received for delay between "
                        "frames.\n",
                        driverName, functionName, laux);
                return asynError;
            }
            setDoubleParam(SDDelayTime, DetTime);


            /* Get trigger modes */
            aux = writeReadNumeric<epicsInt32>("-get conttrig");
            if (aux < 0) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s%s: Invalid value %d received for trigger.\n",
                        driverName, functionName, aux);
                return asynError;
            } else if (aux == 1) {
                setIntegerParam(SDTrigger, 2);
            } else {
                aux = writeReadNumeric<epicsInt32>("-get trig");
                if (aux < 0) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s%s: Invalid value %d received for trigger.\n",
                            driverName, functionName, aux);
                    return asynError;
                } else if (aux == 1) {
                    setIntegerParam(SDTrigger, 1);
                } else {
                    setIntegerParam(SDTrigger, 0);
                }
            }
        }

        callParamCallbacks();
        return asynSuccess;

    } catch (const MythenConnectionError& e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s%s: Connection error happened while reading setting: %s.\n",
                driverName, functionName, e.what());
        return asynError;
    }
}

void acquisitionTaskC(void *drvPvt)
{
    mythen *pPvt = (mythen*)drvPvt;
    pPvt->acquisitionTask();
}

void mythen::acquisitionTask() // TODO Refactor as this is unreadable
{
    static const char *functionName = "acquisitionTask";

    this->lock();
    while (1) {
        epicsInt32 acquire;
        epicsInt32 eventStatus = getStatus();

        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (not acquire || not acquiring_) { // TODO: This OR inbetween seems wrong
            int dataOK;
            size_t nread_expect;
            std::string read_cmd;

            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for acquire to start\n", driverName, functionName);
            this->unlock();
            eventStatus = epicsEventWait(startEventId_);

            if (readmode_ == 0) {       //Raw Mode
                nread_expect = sizeof(epicsInt32)*nmodules_*(1280/chanperline_);
                read_cmd = "-readoutraw";
            } else {
                nread_expect = sizeof(epicsInt32)*nmodules_*(1280);
                read_cmd = "-readout";
            }

            dataOK = 1;
            eventStatus = getStatus();
            setIntegerParam(ADStatus, eventStatus);

            if (eventStatus != ADStatusError) {
                size_t nread;
                size_t nwrite;
                asynStatus status = asynSuccess;
                epicsFloat64 acquireTime;
                epicsUInt32 *detArray = (epicsUInt32*) malloc(nmodules_*1280*sizeof(epicsInt32));

                getDoubleParam(ADAcquireTime, &acquireTime);

                // Work on the cases of what are you getting from getstatus
                do {
                    int eomReason;

                    status = pasynOctetSyncIO->writeRead(pasynUserMeter_,
                            read_cmd.c_str(), sizeof read_cmd.c_str(), (char *)detArray,
                            nread_expect, M1K_TIMEOUT+acquireTime, &nwrite,
                            &nread, &eomReason);

                    if(nread == nread_expect) {
                        this->lock();
                        dataOK = dataCallback(detArray);
                        this->unlock();
                        if (not dataOK) {
                            eventStatus = getStatus();
                            setIntegerParam(ADStatus, eventStatus);
                        }

                    } else {
                        setIntegerParam(ADStatus, getStatus());
                    }

                    if(status != asynSuccess) {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                                "%s:%s: error using readout command status=%d, nRead=%d, eomReason=%d\n",
                                driverName, functionName, status, (int)nread, eomReason);
                    }
                } while (status == asynSuccess && (eventStatus==ADStatusAcquire||eventStatus==ADStatusReadout) && acquiring_);
                free (detArray);
            }
            this->lock();
        }
        if (eventStatus != ADStatusError ) {
            epicsInt32 imageMode;

            printf("Acquisition finish\n");
            getIntegerParam(ADImageMode, &imageMode);
            if (imageMode == ADImageSingle || imageMode == ADImageMultiple) {
                printf("ADAcquire Callback\n");
                acquiring_ = 0;
                setIntegerParam(ADAcquire,  0);
                callParamCallbacks();
            }
        } else {
            //Abort read
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error timed out waiting for data\n",
                    driverName, functionName);
            acquiring_ = 0;
            setAcquire(0);
            setIntegerParam(ADAcquire,  0);
            callParamCallbacks();
        }
    }
}

epicsInt32 mythen::dataCallback(epicsUInt32 *pData)
{
    NDArray *pImage;
    int ndims = 1;
    size_t dims[2];
    int totalBytes;
    int arrayCallbacks;
    int imageCounter;
    epicsTimeStamp timeStamp;
    epicsInt32 colorMode = NDColorModeMono;

    if (pData == NULL || pData[0] < 0) return(0);

    dims[0] = MAX_DIMS;
    dims[1] = 1;
    totalBytes = nmodules_*MAX_DIMS*sizeof(epicsUInt32);

    /* Get the current time */
    epicsTimeGetCurrent(&timeStamp);

    /* Allocate a new image buffer */
    pImage = this->pNDArrayPool->alloc(ndims, dims, NDInt32, totalBytes, NULL);
    if (readmode_==0) {
        decodeRawReadout(nmodules_, nbits_, pData, (epicsUInt32 *)pImage->pData);
    } else {
        decodeRawReadout(nmodules_, 24, pData, (epicsUInt32 *)pImage->pData);
    }

    pImage->dataType = NDUInt32;
    pImage->ndims = ndims;
    pImage->dims[0].size = dims[0];
    pImage->dims[0].offset = 0;
    pImage->dims[0].binning = 1;
    pImage->dims[1].size = dims[1];
    pImage->dims[1].offset = 0;
    pImage->dims[1].binning = 1;

    pImage->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);

    /* Increase image counter */
    getIntegerParam(NDArrayCounter, &imageCounter);
    imageCounter++;
    setIntegerParam(NDArrayCounter, imageCounter);

    /* Set the uniqueId and time stamp */
    pImage->uniqueId = imageCounter;
    pImage->timeStamp = timeStamp.secPastEpoch + timeStamp.nsec / 1e9;

    /* Get any attributes that have been defined for this driver */
    this->getAttributes(pImage->pAttributeList);

    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    if (arrayCallbacks) {
        /* Call the NDArray callback */
        /* Must release the lock here, or we can get into a deadlock, because we can
         * block on the plugin lock, and the plugin can be calling us */
        this->unlock();
        doCallbacksGenericPointer(pImage, NDArrayData, 0);
        this->lock();
    }

    /* We save the most recent good image buffer so it can be used in the
     * readADImage function.  Now release it. */
    if (this->pArrays[0]) this->pArrays[0]->release();
    this->pArrays[0] = pImage;

    /* Update any changed parameters */
    callParamCallbacks();

    return(1);
}

// Input
// nmods: number of active modules
// nbits: number of bits, which were read out
// data: response of the -readoutraw command
//
// Output
// result: array of size 1280*nmods with the number of counts of all
//         channels
void mythen::decodeRawReadout(int nmods, int nbits, epicsUInt32 *data, epicsUInt32 *result)
{
    int chanperline;
    int mask;

    switch (nbits) {
        case 16:
            chanperline = 2;
            mask=0xffff;
            break;
        case 8:
            chanperline = 4;
            mask=0xff;
            break;
        case 4:
            chanperline = 8;
            mask=0xf;
            break;
        case 24: // default to 24 bits
        default:
            chanperline = 1;
            mask = 0xffffff;
    }

    int size = 1280/chanperline*nmods;
    epicsUInt32* tmpArr = (epicsUInt32 *)malloc(size*sizeof(epicsUInt32));
    memcpy(tmpArr, data, size*sizeof(epicsUInt32));
    for (int j = 0; j < chanperline; ++j) {
        int shift = nbits*j;
        int shiftedMask = mask<<shift;
        for (int i = 0; i < size; ++i) {
            result[i*chanperline+j]=((tmpArr[i]&shiftedMask)>>shift)&mask;
        }
    }
    free(tmpArr);
}


/** Called when asyn clients call pasynOctet->write().
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write.
 * \param[in] nChars Number of characters to write
 * \param[out] nActual Number of characters actually written */
asynStatus mythen::writeOctet(asynUser *pasynUser, const char *value,
        size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    const char *functionName = "writeOctet";

    /* Set the parameter in the parameter library. */
    status |= (asynStatus)setStringParam(function, (char *)value);

    /* If this is not a parameter we have handled call the base class */
    if (function < FIRST_SD_PARAM) {
        status |= ADDriver::writeOctet(pasynUser, value,nChars, nActual);
    }

    /* Update any changed parameters */
    callParamCallbacks();

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s: error, status=%d function=%d, value=%s\n",
                driverName, functionName, (asynStatus)status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%s\n",
                driverName, functionName, function, value);
    }

    *nActual = nChars;
    return (asynStatus)status;
}

/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus mythen::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    static const char *functionName = "writeInt32";

    /* Reject any call to the detector if it is running */
    if ((function != ADAcquire) && acquiring_) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: detector is busy\n", driverName, functionName);
        return asynError;
    }

    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setIntegerParam(function, value);

    if (function == ADAcquire) {
        getIntegerParam(SDReadMode, &readmode_);
        status |= setAcquire(value);
    } else if (function == SDSetting) {
        status |= loadSettings(value);
    } else if (function == SDUseFlatField) {
        status |= setFCorrection(value);
    } else if (function == SDUseCountRate) {
        status |= setRCorrection(value);
    } else if (function == SDUseBadChanIntrpl) {
        status |= setBadChanIntrpl(value);
    } else if (function == SDBitDepth) {
        status |= setBitDepth(value);
    } else if (function == SDNumGates) {
        status |= setNumGates(value);
    } else if (function == SDUseGates) {
        status |= setUseGates(value);
    } else if (function == SDTrigger) {
        status |= setTrigger(value);
    } else if (function == SDReset) {
        status |= setReset();
    } else if (function < FIRST_SD_PARAM) {
        /* If this is not a parameter we have handled call the base class */
        status |= ADDriver::writeInt32(pasynUser, value);
    } else {
        status = asynError;
    }

    status |= getSettings();

    /* Update any changed parameters */
    status |= callParamCallbacks();

    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s: error, status=%d function=%d, value=%d\n",
                driverName, functionName, (asynStatus)status, function, value);
        return asynError;
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%d\n",
                driverName, functionName, function, value);
    }

    return asynSuccess;
}

/** Called when asyn clients call pasynFloat64->write().
 * For all  parameters it  sets the value in the parameter library and calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus mythen::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    int addr = 0;
    const char* functionName = "writeFloat64";

    status = getAddress(pasynUser, &addr);
    if (status != asynSuccess) return((asynStatus)status);

    /* Reject any call to the detector if it is running */
    if (acquiring_) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: detector is busy\n", driverName, functionName);
        return asynError;
    }

    /* Set the parameter in the parameter library. */
    status = setDoubleParam(addr, function, value);

    if (function == ADAcquireTime) {
        status |= setExposureTime(value);
    } else if (function == SDDelayTime) {
        status |= setDelayAfterTrigger(value);
    } else if (function == SDThreshold) {
        status |= setKthresh(value);
    } else if (function == SDEnergy) {
        status |= setEnergy(value);
    } else if (function == SDTau) {
        status |= setTau(value);
    } else if (function < NUM_SD_PARAMS) {
        /* If this is not a parameter we have handled call the base class */
        status = ADDriver::writeFloat64(pasynUser, value);
    } else {
        status = asynError;
    }

    status |= getSettings();

    /* Update any changed parameters */
    status |= callParamCallbacks();

    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s: error, status=%d function=%d, value=%g\n",
                driverName, functionName, (asynStatus)status, function, value);
        return asynError;
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%g\n",
                driverName, functionName, function, value);
    }
    return asynSuccess;
}


/** Report status of the driver.
 * Prints details about the driver if details>0.
 * It then calls the ADDriver::report() method.
 * \param[in] fp File pointed passed by caller where the output is written to.
 * \param[in] details If >0 then driver details are printed.
 */
void mythen::report(FILE *fp, int details)
{
    fprintf(fp, "mythen %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}


/** Constructor for mythen driver; most parameters are simply passed to ADDriver::ADDriver.
 * After calling the base class constructor this method creates a thread to collect the detector data,
 * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver, and ADDriver.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] IPPortName The asyn network port connection to the Mythen
 * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
 * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 */
mythen::mythen(const char *portName, const char *IPPortName,
        int maxBuffers, size_t maxMemory,
        int priority, int stackSize)

: ADDriver(portName, 1, NUM_SD_PARAMS, maxBuffers, maxMemory,
        0, 0,             /* No interfaces beyond those set in ADDriver.cpp */
        ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=1, autoConnect=1 */
        priority, stackSize)
{
    int status = asynSuccess;
    const char *functionName = "mythen";

    /* Create the epicsEvents for signaling to the mythen task when acquisition starts and stops */
    this->startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (not this->startEventId_) {
        printf("%s:%s epicsEventCreate failure for start event\n",
                driverName, functionName);
        return;
    }

    // Connect to the server
    status = pasynOctetSyncIO->connect(IPPortName, 0, &pasynUserMeter_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect, status=%d, error=%s\n",
                driverName, functionName, (asynStatus)status, 
                pasynUserMeter_->errorMessage);
        return;
    }

    createParam(SDSettingString,          asynParamInt32,   &SDSetting);
    createParam(SDDelayTimeString,        asynParamFloat64, &SDDelayTime);
    createParam(SDThresholdString,        asynParamFloat64, &SDThreshold);
    createParam(SDEnergyString,           asynParamFloat64, &SDEnergy);
    createParam(SDUseFlatFieldString,     asynParamInt32,   &SDUseFlatField);
    createParam(SDUseCountRateString,     asynParamInt32,   &SDUseCountRate);
    createParam(SDUseBadChanIntrplString, asynParamInt32,   &SDUseBadChanIntrpl);
    createParam(SDBitDepthString,         asynParamInt32,   &SDBitDepth);
    createParam(SDUseGatesString,         asynParamInt32,   &SDUseGates);
    createParam(SDNumGatesString,         asynParamInt32,   &SDNumGates);
    createParam(SDNumFramesString,        asynParamInt32,   &SDNumFrames);
    createParam(SDTriggerString,          asynParamInt32,   &SDTrigger);
    createParam(SDResetString,            asynParamInt32,   &SDReset);
    createParam(SDTauString,              asynParamFloat64, &SDTau);
    createParam(SDNModulesString,         asynParamInt32,   &SDNModules);
    createParam(SDFirmwareVersionString,  asynParamOctet,   &SDFirmwareVersion);
    createParam(SDReadModeString,         asynParamInt32,   &SDReadMode);

    status =  setStringParam (ADManufacturer, "Dectris");
    status |= setStringParam (ADModel,        "Mythen");

    status |= getFirmware();
    status |= setStringParam (SDFirmwareVersion, fwVersion_.version_str.c_str());

    int sensorSizeX = MAX_DIMS;
    int sensorSizeY = 1;
    status |= setIntegerParam(ADMaxSizeX, sensorSizeX);
    status |= setIntegerParam(ADMaxSizeY, sensorSizeY);

    int minX = 1;
    int minY = 1;
    int sizeX = MAX_DIMS;
    int sizeY = 1;
    status |= setIntegerParam(ADMinX,  minX);
    status |= setIntegerParam(ADMinY,  minY);
    status |= setIntegerParam(ADSizeX, sizeX);
    status |= setIntegerParam(ADSizeY, sizeY);

    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType,  NDInt32);

    status |= setIntegerParam(ADImageMode, ADImageSingle);

    /* NOTE: these char type waveform record could not be initialized in iocInit
     * Instead use autosave to restore their values.
     * It is left here only for references.
     * */
    status |= setIntegerParam(ADStatus, getStatus());

    // Read the current settings from the device.  This will set parameters in the parameter library.
    getSettings();

    int aux = 0;
    try {
        aux = writeReadNumeric<epicsInt32>("-get nmodules");
        if (aux < 0) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error, -get nmodules\n",
                    driverName, functionName);
            return;
        }
        nmodules_ = aux;
        status |= setIntegerParam(SDNModules, aux);
    } catch (const MythenConnectionError& e) {
        status |= asynError;
    }

    callParamCallbacks();

    if (status != asynSuccess) {
        printf("%s: unable to read camera parameters\n", functionName);
        return;
    }

    /* Create the thread that runs acquisition */
    epicsThreadCreate("acquisitionTask",
                epicsThreadPriorityMedium,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)acquisitionTaskC,
                this);
}

} // namespace mythen

extern "C" int mythenConfig(const char *portName, const char *IPPortName,
        int maxBuffers, size_t maxMemory,
        int priority, int stackSize)
{
    new mythen::mythen(portName, IPPortName,
            maxBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
}

/* Code for iocsh registration */
static const iocshArg mythenConfigArg0 = {"Port name", iocshArgString};
static const iocshArg mythenConfigArg1 = {"Asyn port name", iocshArgString};
static const iocshArg mythenConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg mythenConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg mythenConfigArg4 = {"priority", iocshArgInt};
static const iocshArg mythenConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const mythenConfigArgs[] =  {&mythenConfigArg0,
                                                     &mythenConfigArg1,
                                                     &mythenConfigArg2,
                                                     &mythenConfigArg3,
                                                     &mythenConfigArg4,
                                                     &mythenConfigArg5};
static const iocshFuncDef configmythen = {"mythenConfig", 6, mythenConfigArgs};
static void configmythenCallFunc(const iocshArgBuf *args)
{
    mythenConfig(args[0].sval, args[1].sval, args[2].ival,
            args[3].ival, args[4].ival,  args[5].ival);
}


static void mythenRegister(void)
{
    iocshRegister(&configmythen, configmythenCallFunc);
}

extern "C" {
    epicsExportRegistrar(mythenRegister);
}

