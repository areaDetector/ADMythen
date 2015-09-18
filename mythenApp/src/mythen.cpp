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

// #include "mythenV3.h"

#define MAX_FILENAME_LEN 256
#define MAX_DIMS      1280
#define MAX_COMMAND_LEN 128
#define MAX_NMODULES 2
#define M1K_TIMEOUT 5.0
#define MAX_FRAMES 500
#define MAX_TRIGGER_TIMEOUT_COUNT 50

static const char *driverName = "mythen";

#define SDSettingString         "SD_SETTING"
#define SDDelayTimeString       "SD_DELAY_TIME"
#define SDThresholdString       "SD_THRESHOLD"
#define SDEnergyString          "SD_ENERGY"
#define SDUseFlatFieldString    "SD_USE_FLATFIELD"
#define SDUseCountRateString    "SD_USE_COUNTRATE"
#define SDTauString             "SD_TAU"
#define SDUseBadChanIntrplString "SD_USE_BADCHANNEL_INTRPL"
#define SDBitDepthString        "SD_BIT_DEPTH"
#define SDUseGatesString        "SD_USE_GATES"
#define SDNumGatesString        "SD_NUM_GATES"
#define SDNumFramesString       "SD_NUM_FRAMES"
#define SDTriggerString         "SD_TRIGGER"
#define SDResetString           "SD_RESET"
#define SDNModulesString        "SD_NMODULES"
#define SDFirmwareVersionString "SD_FIRMWARE_VERSION"  /* asynOctet    ro */
#define SDReadModeString        "SD_READ_MODE"


/** Driver for sls array detectors using over TCP/IP socket */
class mythen : public ADDriver {
public:
    mythen(const char *portName, const char *IPPortName,
                   int maxBuffers, size_t maxMemory,
                   int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual);
    virtual void report(FILE *fp, int details); 

    epicsInt32 dataCallback(epicsInt32 *pData); /* This should be private but is called from C so must be public */
    void pollTask(); 
    void acquisitionTask(); 
    void shutdown(); 
 
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
    int SDNumCycles; 
    int SDNumFrames; 
    int SDTrigger;
    int SDReset;
    int SDTau;
    int SDFirmwareVersion;
    int SDReadMode;
    int SDNModules;
    #define LAST_SD_PARAM SDNModules

    /* These are the methods we implement from Mythen */
    virtual asynStatus setAcquire(epicsInt32 value);
    virtual asynStatus setFCorrection(epicsInt32 value);
    virtual asynStatus setRCorrection(epicsInt32 value);
    virtual asynStatus setExposureTime(epicsFloat64 value);
    virtual asynStatus setDelayAfterTrigger(epicsFloat64 value);
    virtual asynStatus setBitDepth(epicsInt32 value);
    virtual asynStatus setBadChanIntrpl(epicsInt32 value);
    virtual asynStatus setUseGates(epicsInt32 value);
    virtual asynStatus setNumGates(epicsInt32 value);
    virtual asynStatus setKthresh(epicsFloat64 value);
    virtual asynStatus setEnergy(epicsFloat64 value);
    virtual asynStatus setTau(epicsFloat64 value);
    virtual asynStatus setFrames(epicsInt32 value);
    virtual asynStatus setFlip(epicsInt32 value);
    virtual asynStatus setTrigger(epicsInt32 value);
    virtual asynStatus loadSettings(epicsInt32 value);
    virtual asynStatus setReset();
    virtual asynStatus getSettings();
    virtual epicsInt32  getStatus();
    virtual asynStatus getFirmware();
    void decodeRawReadout(int nmods, int nbits, int *data, int *result);

 private:                                       
    /* These are the methods that are new to this class */

    /* Our data */
    epicsEventId startEventId_;
    asynUser *pasynUserMeter_;
    epicsInt32 acquiring_;
    epicsInt32 frames_;
    epicsInt32 chanperline_, nbits_;
    epicsInt32 nmodules, readmode_;
    char *IPPortName_;
    char firmwareVersion_[7];
    char outString_[MAX_COMMAND_LEN];
    char inString_[MAX_COMMAND_LEN];
    bool isBigEndian_;
    epicsInt32 *detArray_;
    epicsUInt32 *tmpArray_;
    asynStatus sendCommand();
    asynStatus writeReadMeter();
    epicsInt32 stringToInt32(char *str);
    long long stringToInt64(char *str);
    epicsFloat32 stringToFloat32(char *str);
    void swap4(char* value);
    void swap8(char* value);
};

#define NUM_SD_PARAMS (&LAST_SD_PARAM - &FIRST_SD_PARAM + 1)

void mythen::swap4(char *value)
{
    char temp;
    temp = value[0];
    value[0] = value[3];
    value[3] = temp;
    temp = value[1];
    value[1] = value[2];
    value[2] = temp;
}

void mythen::swap8(char *value)
{
    char temp;
    temp = value[0];
    value[0] = value[7];
    value[7] = temp;
    temp = value[1];
    value[1] = value[6];
    value[6] = temp;
    temp = value[2];
    value[2] = value[5];
    value[5] = temp;
    temp = value[3];
    value[3] = value[4];
    value[4] = temp;
}

epicsInt32 mythen::stringToInt32(char *str)
{
    epicsInt32 value = *reinterpret_cast<epicsInt32*>(str);
    if (isBigEndian_) swap4((char *)&value);
    return value;
}

long long mythen::stringToInt64(char *str)
{
    long long value = *reinterpret_cast<long long*>(str);
    if (isBigEndian_) swap8((char *)&value);
    return value;
}

epicsFloat32 mythen::stringToFloat32(char *str)
{
    epicsFloat32 value = *reinterpret_cast<epicsFloat32*>(str);
    if (isBigEndian_) swap4((char *)&value);
    return value;
}

/** Sends a command to the detector and reads the response.**/
asynStatus mythen::sendCommand()
{
  asynStatus status;
  const char *functionName="sendCommand";
  int aux;

  status = writeReadMeter();
  aux = stringToInt32(this->inString_);
  //check for errors
  if (aux < 0){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: error, expected 0, received %d\n",
          driverName, functionName, aux);
  }
  return status;
}

/** Send a string to the detector and reads the response.**/
asynStatus mythen::writeReadMeter()
{
    size_t nread;
    size_t nwrite;
    asynStatus status;
    int eomReason;
    const char *functionName="writeReadMeter";

    if(strcmp(outString_,"-get tau")==0) {
     status = pasynOctetSyncIO->writeRead(pasynUserMeter_, outString_, strlen(outString_),
         inString_, sizeof(epicsFloat32), M1K_TIMEOUT, &nwrite, &nread, &eomReason);
    }
    else {
        if (strcmp(outString_,"-get version")==0) {
          status = pasynOctetSyncIO->writeRead(pasynUserMeter_, outString_, strlen(outString_),
                    firmwareVersion_, sizeof(firmwareVersion_), M1K_TIMEOUT, &nwrite, &nread, &eomReason);
        }
        else {
          status = pasynOctetSyncIO->writeRead(pasynUserMeter_, outString_, strlen(outString_),
                inString_, sizeof(int), M1K_TIMEOUT, &nwrite, &nread, &eomReason);
        }
    }
    if(status != asynSuccess)
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: error!\n",driverName, functionName);
    return status;
}

/** Starts and stops the acquisition. **/
asynStatus mythen::setAcquire(epicsInt32 value)
{
    size_t nread;
    size_t nwrite;
    asynStatus status = asynSuccess;
    epicsInt32 eomReason;
    static const char *functionName = "setAcquire";
    // printf("setAcquire %d\n",value);
    if (value == 0) {
      //        while (1) {
      //            // Repeat sending STOP until we get only a 0 back
      //            status = pasynOctetSyncIO->setInputEos(pasynUserMeter_, "", 0);
      //            if (status) break;
      status = pasynOctetSyncIO->writeRead(pasynUserMeter_, "-stop", strlen(outString_),
                       inString_, sizeof(epicsInt32), M1K_TIMEOUT, &nwrite, &nread, &eomReason);
            setIntegerParam(ADStatus, getStatus());
        //            if (status == asynSuccess || status == asynTimeout) break;
        //        }
      acquiring_ = 0;
    } else {
        if(!(acquiring_)) {                 
          strcpy(outString_,"-start");
          this->sendCommand();
          // Notify the read thread that acquisition has started
                epicsEventSignal(startEventId_);
          
          acquiring_ = 1;
        }
    }
    if (status) {
        acquiring_ = 0;
    }
    if(status != asynSuccess)
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error!\n", driverName, functionName);
    return status;
}

/** Sets the number of frames
 * whitin an acquisition**/
asynStatus mythen::setFrames(epicsInt32 value)
{
    asynStatus status;
    epicsInt32 imageMode;
    getIntegerParam(ADImageMode, &imageMode);
    if (imageMode ==0)  //checks if it is in single acquire mode
    {
      epicsSnprintf(outString_, sizeof(outString_), "-frames %d", 1);
    }
    else
    {
      epicsSnprintf(outString_, sizeof(outString_), "-frames %d", value);
    }
    status = sendCommand();

    // Save frame count for acquition    
    frames_ = value;

    return status;
}

/** Sets the Trigger Mode
 *      0 = None, 1 = Single, 2=Continuous */
asynStatus mythen::setTrigger(epicsInt32 value)
{
    asynStatus status = asynSuccess;;

    if (value == 0) {
      // Clearing trigen clears conttrig also
      epicsSnprintf(outString_, sizeof(outString_), "-trigen 0");
      status = sendCommand();
    }
    else if (value == 1) {
      epicsSnprintf(outString_, sizeof(outString_), "-trigen 1");
      status = sendCommand();
    }
    else if (value == 2) {
      epicsSnprintf(outString_, sizeof(outString_), "-conttrigen 1");
      status = sendCommand();
    }

    return status;
}


/** Sets the dead time constant for the rate correction**/
asynStatus mythen::setTau(epicsFloat64 value)
{
    asynStatus status;
    if(value == -1 || value > 0) {
        epicsSnprintf(outString_, sizeof(outString_), "-tau %f", value);
        status = sendCommand();
    }
    else {
        setDoubleParam(SDTau,0);
        callParamCallbacks();

        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "error check if tau -1 or >0 (outString = %s)",outString_);
        return asynError;
    }
    return status;
}


/** Sets the energy threshold for the module **/
asynStatus mythen::setKthresh(epicsFloat64 value)
{
    epicsInt32 i;
    asynStatus status = asynSuccess;
    for(i=0;i<this->nmodules;i++) {
        epicsSnprintf(outString_, sizeof(outString_), "-module %d",i);
        status = sendCommand();
        epicsSnprintf(outString_, sizeof(outString_), "-kthresh %f", value);
        status = sendCommand();
    }
    if(status == asynSuccess) {
        setDoubleParam(SDThreshold,value);
        callParamCallbacks();
    }
    else {
        setDoubleParam(SDThreshold,0);
        callParamCallbacks();

        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "error check if value > 0 (outString = %s)",outString_);
        return asynError;
    }
    return status;
}


/** Sets the energy threshold for the module **/
asynStatus mythen::setEnergy(epicsFloat64 value)
{
    
    epicsInt32 i;
  asynStatus status = asynSuccess;
  if ((int)firmwareVersion_[1]%48 >=3) {
    for(i=0;i<this->nmodules;i++) {
        epicsSnprintf(outString_, sizeof(outString_), "-module %d",i);
        status = sendCommand();
        epicsSnprintf(outString_, sizeof(outString_), "-energy %f", value);
        status = sendCommand();
    }
    if(status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "error check if value > 0 (outString = %s)",outString_);
        return asynError;
    }
  }
  return status;
}

/** Sets the exposure time of one frame. (units of 100ns) **/
asynStatus mythen::setExposureTime(epicsFloat64 value)
{
    asynStatus status;
    int hns = (int)(value * (1E+7));
    epicsSnprintf(outString_, sizeof(outString_), "-time %d", hns);
    status = sendCommand();
    return status;
}

/** Sets the exposure time of one frame. (units of 100ns) **/
asynStatus mythen::setDelayAfterTrigger(epicsFloat64 value)
{
    asynStatus status;
    int hns = (int)(value * (1E+7));
    epicsSnprintf(outString_, sizeof(outString_), "-delafter %d", hns);
    status = sendCommand();
    return status;
}

/** Enables or disables the flatfield correction. 
    After initialisation the flatfield correction is enabled. **/
asynStatus mythen::setFCorrection(epicsInt32 value)
{
    asynStatus status;

    epicsSnprintf(outString_, sizeof(outString_), "-flatfieldcorrection %d", value);
    status = sendCommand();
    return status;
}

/** Enables or disables the bad channel interpolation **/
asynStatus mythen::setBadChanIntrpl(epicsInt32 value)
{
    asynStatus status;

    epicsSnprintf(outString_, sizeof(outString_), "-badchannelinterpolation %d", value);
    status = sendCommand();
    return status;
}


/** Enables or disables the rate correction. 
    After initialisation the rate correction is disabled. **/
asynStatus mythen::setRCorrection(epicsInt32 value)
{
    asynStatus status;

    epicsSnprintf(outString_, sizeof(outString_), "-ratecorrection %d", value);
    status = sendCommand();
    return status;
}

/** Enables or disables the gates. 
    After initialisation the gates are disabled. **/
asynStatus mythen::setUseGates(epicsInt32 value)
{
    asynStatus status;

    epicsSnprintf(outString_, sizeof(outString_), "-gateen %d", value);
    status = sendCommand();
    return status;
}

/** Number of gates. **/
asynStatus mythen::setNumGates(epicsInt32 value)
{
    asynStatus status;

    epicsSnprintf(outString_, sizeof(outString_), "-gates %d", value);
    status = sendCommand();
    return status;
}

/** Get Firmware Version **/
asynStatus mythen::getFirmware()
{
    asynStatus status=asynSuccess;
    
    epicsSnprintf(outString_, sizeof(outString_), "-get version");
    writeReadMeter();
    //epicsSnprintf(inString_, sizeof(inString_), "");
    
    return status;
}

/** Get Acquition Status */ 
epicsInt32 mythen::getStatus()
{
    epicsInt32 detStatus;
    int aux;

    strcpy(outString_, "-get status");
    writeReadMeter();
    aux = stringToInt32(this->inString_);
    int m_status =  aux & 1;          // Acquire running status (non-zero)
    int t_status = aux & (1<<3);      // Waiting for trigger (non-zero)        
    int d_status = aux & (1<<16);     // No Data Available when not zero
    int triggerWaitCnt=0;
    double triggerWait;
    
    //printf("Mythen Status - M:%d\tT:%d\tD:%d\n",m_status,t_status, d_status);
    
    if (m_status || !d_status ) { 
      detStatus = ADStatusAcquire;
      
      triggerWaitCnt=0;
      //Waits for Trigger for increaseing amount of time for a total of almost 1 minute
      while ((t_status ) && (triggerWaitCnt<MAX_TRIGGER_TIMEOUT_COUNT)) {
        triggerWait = 0.0001*pow(10.0,((double)(triggerWaitCnt/10)+1));
        //Wait
        epicsThreadSleep(triggerWait);
        //Check again
        strcpy(outString_, "-get status");
        writeReadMeter();
        aux = stringToInt32(this->inString_);
        t_status = aux & (1<<3);       
        d_status = aux & (1<<16);     
        triggerWaitCnt++;
        
      }
      
      if (!d_status)
        detStatus = ADStatusReadout;
      if (triggerWaitCnt==MAX_TRIGGER_TIMEOUT_COUNT)
        detStatus = ADStatusError;
    }
    else
      detStatus = ADStatusIdle;
    
    return detStatus;
}


/**Enables or disables the flipping of the channel numbering. **/
asynStatus mythen::setFlip(epicsInt32 value)
{
    asynStatus status;

    epicsSnprintf(outString_, sizeof(outString_), "-flipchannels %d", value);
    status = sendCommand();
    return status;
}

/**Enables or disables the flipping of the channel numbering. **/
asynStatus mythen::setBitDepth(epicsInt32 value)
{
    asynStatus status;
    epicsInt32 nbits;
    switch (value) {
      case 0: 
        nbits = 24;
        break;
      case 1: 
        nbits = 16;
        break;
      case 2:
        nbits = 8;
        break;
      case 3:
        nbits = 4;
        break;
      default:
        nbits = 24;
        break;
    }

    epicsSnprintf(outString_, sizeof(outString_), "-nbits %d", nbits);
    status = sendCommand();
    return status;
}

/**Loads predefined settings for the current module to measure
some common x-ray radiation. The command loads the energy calibration,
the bad channels file, the flatfield correction, and the trimbits
for the corresponding settings, and sets the energy threshold to a
suitable value.**/
asynStatus mythen::loadSettings(epicsInt32 value)
{
    asynStatus status=asynSuccess;
    epicsInt32 i=0;

    for(i=0;i<this->nmodules;i++) {
        epicsSnprintf(outString_, sizeof(outString_), "-module %d",i);
        status = sendCommand();

        switch(value){
            //Cu
            case 0:
                if ((int)firmwareVersion_[1]%48 >=3) {
                    epicsSnprintf(outString_, sizeof(outString_), "-settings Cu");
                }
                else {
                    epicsSnprintf(outString_, sizeof(outString_), "-settings StdCu");
                }
                break;

            //Mo
            case 1:  
                if ((int)firmwareVersion_[1]%48 >=3) {
                    epicsSnprintf(outString_, sizeof(outString_), "-settings Mo");
                  }
                else {
                    epicsSnprintf(outString_, sizeof(outString_), "-settings StdMo");
                  }
                break;

            //Ag
            //Not supported on firmware verions less than 3
            case 2:
                epicsSnprintf(outString_, sizeof(outString_), "-settings Ag");
                break;
                
            // Cr
            case 3:                
                if ((int)firmwareVersion_[1]%48 >=3) {
                    epicsSnprintf(outString_, sizeof(outString_), "-settings Cr");
                }
                else {
                    epicsSnprintf(outString_, sizeof(outString_), "-settings HgCr");
                }
                break;
                
            default:
                epicsSnprintf(outString_, sizeof(outString_), "-reset");
                break;
        }
        status = sendCommand();
    }
    setIntegerParam(SDSetting,value);

    callParamCallbacks();

    return status;
}

/**Sets the detector back to default settings. This command takes
about two seconds per module.**/
asynStatus mythen::setReset()
{
    asynStatus status=asynSuccess;
    epicsInt32 i=0;

    setIntegerParam(SDReset,1);
    for(i=0;i<this->nmodules;i++) {
        epicsSnprintf(outString_, sizeof(outString_), "-module %d",i);
        status = sendCommand();

        strcpy(outString_, "-reset");
        status = sendCommand();
    }
    setIntegerParam(SDReset,0);
    callParamCallbacks();
    return status;
}


/** Reads the values of all the modules parameters, sets them in the parameter library**/
asynStatus mythen::getSettings()
{
    int aux;
    epicsFloat32 faux;
    long long laux;
    epicsFloat64 DetTime;

    static const char *functionName = "getSettings";

    if (acquiring_) {
        strcpy(outString_,"Called during Acquire");
        inString_[0] = 0;
        goto error;
    }

    strcpy(outString_, "-get flatfieldcorrection");
    writeReadMeter();
    aux = stringToInt32(this->inString_);
    if (aux!=0 && aux!=1) goto error;
    setIntegerParam(SDUseFlatField, aux);


    strcpy(outString_, "-get badchannelinterpolation");
    writeReadMeter();
    aux = stringToInt32(this->inString_);
    if (aux!=0 && aux!=1) goto error;
    setIntegerParam(SDUseBadChanIntrpl, aux);

    strcpy(outString_, "-get ratecorrection");
    writeReadMeter();
    aux = stringToInt32(this->inString_);
    if (aux!=0 && aux!=1) goto error;
    setIntegerParam(SDUseCountRate, aux);

    strcpy(outString_, "-get nbits");
    writeReadMeter();
    aux = stringToInt32(this->inString_);
    if (aux < 0) goto error;
    nbits_ = aux;
    chanperline_ = 32/aux; 
    setIntegerParam(SDBitDepth, aux);

    strcpy(outString_, "-get time");
    writeReadMeter();
    aux = stringToInt32(this->inString_);
    if (aux >= 0) DetTime = (aux * (1E-7));
    else goto error;
    setDoubleParam(ADAcquireTime,DetTime);

    

    strcpy(outString_, "-get frames");
    writeReadMeter();
    aux = stringToInt32(this->inString_);
    if (aux >= 0) setIntegerParam(SDNumFrames, aux);

    strcpy(outString_, "-get tau");
    writeReadMeter();
    faux = stringToFloat32(this->inString_);
    if (faux == -1 || faux > 0) setDoubleParam(SDTau,faux);
    else goto error;

    

    strcpy(outString_, "-get kthresh");
    writeReadMeter();
    faux = stringToFloat32(this->inString_);
    if (faux < 0) goto error;
    else setDoubleParam(SDThreshold,faux);
      
      
    //Firmware greater than 3 commands  
    if ((int)firmwareVersion_[1]%48 >=3) {
      strcpy(outString_, "-get energy");
      writeReadMeter();
      faux = stringToFloat32(this->inString_);
      if (faux < 0) goto error;
      else setDoubleParam(SDEnergy,faux);
      
      strcpy(outString_, "-get delafter");
      writeReadMeter();
      laux = stringToInt64(this->inString_);
      if (laux >= 0) DetTime = (laux * (1E-7));
      else goto error;
      setDoubleParam(SDDelayTime,DetTime);
      
      
      /* Get trigger modes */
      strcpy(outString_, "-get conttrig");
      writeReadMeter();
      aux = stringToInt32(this->inString_);
      if (aux < 0) goto error;
      if (aux == 1) 
          setIntegerParam(SDTrigger, 2);
      else {
        strcpy(outString_, "-get trig");
        writeReadMeter();
        aux = stringToInt32(this->inString_);
        if (aux < 0) goto error;
        if (aux == 1)
          setIntegerParam(SDTrigger, 1);
        else
          setIntegerParam(SDTrigger, 0);
      }
    }

    callParamCallbacks();
    // if (prevAcquiring) setAcquire(1);

    return asynSuccess;

    error:
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: error, outString=%s, inString=%s\n",
        driverName, functionName, outString_, inString_);
    return asynError;
}


//static void c_shutdown(void* arg) {
//    mythen *p = (mythen*)arg;
//    p->shutdown(); 
//}


void acquisitionTaskC(void *drvPvt)
{
    mythen *pPvt = (mythen*)drvPvt; 
    pPvt->acquisitionTask(); 
}

void pollTaskC(void *drvPvt)
{
    mythen *pPvt = (mythen*)drvPvt; 
    pPvt->pollTask(); 
}

void mythen::shutdown()
{
  //    if (pDetector)
  //        delete pDetector; 
}

void mythen::pollTask()
{
    // int acquire; 
    /* Poll detector running status every second*/
    while (1) {
        epicsThreadSleep(1); 

        /* Update detector status */
        this->lock(); 
        // int detStatus = pDetector->getDetectorStatus();
        // setIntegerParam(ADStatus, detStatus);
        callParamCallbacks(); 
        this->unlock(); 
    }
}

void mythen::acquisitionTask()
{
    size_t nread, nread_expect;
    size_t nwrite;
    int eventStatus;
    int imageMode;
    epicsInt32 acquire, eomReason;
    double acquireTime;
    asynStatus status = asynSuccess;
    int dataOK;

    static const char *functionName = "acquisitionTask";
    this->lock(); 

    while (1) {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);
        
        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire || !acquiring_)  {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
            this->unlock();
            eventStatus = epicsEventWait(this->startEventId_);
            // setStringParam(ADStatusMessage, "Acquiring data");
            // setIntegerParam(ADNumImagesCounter, 0);
              // getIntegerParam(ADAcquire, &acquire);
              
                  
              //printf("Read Mode: %d\tnModules: %d\t chanperline: %d\n", readmode_,this->nmodules,chanperline_);
            if (readmode_==0)       //Raw Mode
                nread_expect = sizeof(epicsInt32)*this->nmodules*(1280/chanperline_);
            else
                nread_expect = sizeof(epicsInt32)*this->nmodules*(1280);
                
            dataOK = 1;

            eventStatus = getStatus();
            setIntegerParam(ADStatus, eventStatus);

            if (eventStatus!=ADStatusError) {

              getDoubleParam(ADAcquireTime,&acquireTime);
              // printf("Acquisition start - expect %d\n",nread_expect);
              // Work on the cases of what are you getting from getstatus
              do {
                nread=0;
                if (readmode_==0)
                  strcpy(outString_, "-readoutraw");
                else
                  strcpy(outString_, "-readout");

                status = pasynOctetSyncIO->writeRead(pasynUserMeter_, outString_, strlen(outString_), (char *)detArray_,
                                        nread_expect, M1K_TIMEOUT+acquireTime, &nwrite, &nread, &eomReason);  //Timeout is M1K_TIMEOUT + AcquireTime

                //printf("nread_expected = %d\tnread = %d\n", nread_expect,nread);

                if(nread == nread_expect) {
                    this->lock();
                    dataOK = dataCallback(detArray_);
                    this->unlock();
                    if (!dataOK) {
                        eventStatus = getStatus();
                        setIntegerParam(ADStatus, eventStatus);
                    }

                }
                else {
                    eventStatus = getStatus();
                    setIntegerParam(ADStatus, eventStatus);
                  //printf("Data not size expected ADStatus: %d\n",eventStatus);
                }
                if(status != asynSuccess) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: error using readout command status=%d, nRead=%d, eomReason=%d\n",
                          driverName, functionName, status, (int)nread, eomReason);
                }
              } 
              while (status == asynSuccess && (eventStatus==ADStatusAcquire||eventStatus==ADStatusReadout) && acquiring_);
             
           }
           this->lock();
            
        }
        if (eventStatus!=ADStatusError ) {
          printf("Acquisition finish\n");
          getIntegerParam(ADImageMode, &imageMode);
          if (imageMode == ADImageSingle || imageMode == ADImageMultiple) {
            printf("ADAcquire Callback\n");
            acquiring_ = 0;
            setIntegerParam(ADAcquire,  0); 
            callParamCallbacks(); 
          }
        }
        else {
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

epicsInt32 mythen::dataCallback(epicsInt32 *pData)
{
    NDArray *pImage; 
    int ndims = 1;
    size_t dims[2];
    int totalBytes; 
    int arrayCallbacks;
    int imageCounter;
    epicsTimeStamp timeStamp; 
    epicsInt32 colorMode = NDColorModeMono;

    // printf ("pData[0] = %X\n",pData[0]);

    if (pData == NULL || pData[0] < 0) return(0); 

    dims[0] = MAX_DIMS;
    dims[1] = 1;
    totalBytes = this->nmodules*MAX_DIMS*sizeof(epicsInt32);

    /* Get the current time */
    epicsTimeGetCurrent(&timeStamp); 

    /* Allocate a new image buffer */
    pImage = this->pNDArrayPool->alloc(ndims, dims, NDInt32, totalBytes, NULL);
    if (readmode_==0)
      decodeRawReadout(this->nmodules, this->nbits_, pData, (int *)pImage->pData);
    else
      decodeRawReadout(this->nmodules, 24, pData, (int *)pImage->pData);
    //    memcpy(pImage->pData,  pData, totalBytes); 
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
// result: array of size 1280*nmods with the number of counts of all // channels
void mythen::decodeRawReadout(int nmods, int nbits, int *data, int *result)
{
  // default for 24 bits
  int chanperline = 1; // default for 24 bits
  int mask=0xffffff; // default for 24 bits
  if (nbits == 16) {
      chanperline = 2;
      mask=0xffff;
  }
  if (nbits == 8) {
      chanperline = 4;
      mask=0xff;
  }
  if (nbits == 4) {
      chanperline = 8;
      mask=0xf;
  }

  int size = 1280/chanperline*nmods;
  memcpy(tmpArray_, data, size*sizeof(int)); // unsigned int32
  for (int j = 0; j < chanperline; j++) {
      int shift = nbits*j;
      int shiftedMask = mask<<shift;
      for (int i = 0; i < size; i++) {
        result[i*chanperline+j]=((tmpArray_[i]&shiftedMask)>>shift)&mask;
    }
  }
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

    /* Reject any call to the detector if it is running */
    //    if (acquiring_) {
    //        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
    //        "%s:%s: detector is busy\n", driverName, functionName);
    //        return asynError;
    //    }

    /* Set the parameter in the parameter library. */
    status |= (asynStatus)setStringParam(function, (char *)value);

    /* If this is not a parameter we have handled call the base class */
    if (function < FIRST_SD_PARAM) 
      status = ADDriver::writeOctet(pasynUser, value,nChars, nActual);
 
    /* Update any changed parameters */
    callParamCallbacks();

    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%s\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%s\n", 
              driverName, functionName, function, value);

    *nActual = nChars;
    return((asynStatus)status); 
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
    }
    else {
      if (function == SDSetting) {
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
        status = setNumGates(value);
      } else if (function == SDUseGates) {
        status = setUseGates(value);
      } else if (function == SDNumFrames) {
        status |= setFrames(value);
      } else if (function == SDTrigger) {
        status |= setTrigger(value);
      } else if (function == SDReset) {
        status |= setReset();
      } else if (function == ADImageMode) {

        //getIntegerParam(SDNumFrames, &frames_);
        status |= setFrames(frames_);
      } else {
        /* If this is not a parameter we have handled call the base class */
        if (function < FIRST_SD_PARAM) status = ADDriver::writeInt32(pasynUser, value);
      }
      status |= getSettings();
    }
 
    /* Update any changed parameters */
    status |= callParamCallbacks();

    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return((asynStatus)status); 
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
    status = (asynStatus) setDoubleParam(addr, function, value);

    if (function == ADAcquireTime) {
      status |= setExposureTime(value);
    } else if (function == SDDelayTime) {
      status |= setDelayAfterTrigger(value);
    } else if (function == SDThreshold) {
        /* note down user's set value and recover it when settings change */
        // threshold = value;
        status |= setKthresh(value);
    } else if (function == SDEnergy) {
        status |= setEnergy(value);
    } else if (function == SDTau) {
        status |= setTau(value);
    } else {
        /* If this is not a parameter we have handled call the base class */
        if (function < NUM_SD_PARAMS) status = ADDriver::writeFloat64(pasynUser, value);
    }

 
    status |= getSettings();
   
    /* Update any changed parameters */
    status |= callParamCallbacks();

    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%g\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%g\n", 
              driverName, functionName, function, value);
    return((asynStatus)status); 
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

extern "C" int mythenConfig(const char *portName, const char *IPPortName,
                                    int maxBuffers, size_t maxMemory,
                                    int priority, int stackSize)
{
    new mythen(portName, IPPortName,
         maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
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
         // , pDetector(NULL)

{
    int status = asynSuccess;
    // NDArray *pData; 
    const char *functionName = "mythen";

    IPPortName_ = epicsStrDup(IPPortName);
    
    isBigEndian_ = EPICS_BYTE_ORDER == EPICS_ENDIAN_BIG;

    /* Create the epicsEvents for signaling to the mythen task when acquisition starts and stops */
    this->startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId_) {
        printf("%s:%s epicsEventCreate failure for start event\n", 
            driverName, functionName);
        return;
    }

    // Connect to the server
    status = pasynOctetSyncIO->connect(IPPortName, 0, &pasynUserMeter_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect, status=%d, error=%s\n",
               driverName, functionName, status, pasynUserMeter_->errorMessage);
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
    status |= setStringParam (SDFirmwareVersion, firmwareVersion_);
    
    int sensorSizeX = MAX_DIMS;
    int  sensorSizeY = 1;
    status |= setIntegerParam(ADMaxSizeX, sensorSizeX);
    status |= setIntegerParam(ADMaxSizeY, sensorSizeY);

    int minX,  minY, sizeX, sizeY; 
    minX = 1; minY = 1; sizeX = MAX_DIMS; sizeY = 1; 
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
    
    //Get Firmware version
    
    
    // Read the current settings from the device.  This will set parameters in the parameter library.
    getSettings();

    int aux;
    //get nmodules and check for errors
    strcpy(outString_, "-get nmodules");
    status |= writeReadMeter();
    aux = stringToInt32(this->inString_);
    if (aux < 0) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: error, outString=%s, inString=%s\n",
        driverName, functionName, outString_, inString_);
      return;
    }
    this->nmodules=aux;
    status |= setIntegerParam(SDNModules, aux);
    detArray_ = (epicsInt32*) calloc(this->nmodules*1280, sizeof(epicsInt32));
    tmpArray_ = (epicsUInt32*) calloc(this->nmodules*1280, sizeof(epicsInt32));

    callParamCallbacks();

    if (status) {
        printf("%s: unable to read camera parameters\n", functionName);
        return;
    }

    /* Register the shutdown function for epicsAtExit */
    // epicsAtExit(c_shutdown, (void*)this); 

    /* Create the thread that runs acquisition */
    status = (epicsThreadCreate("acquisitionTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)acquisitionTaskC,
                                this) == NULL);

    /* Create the thread that polls status */
    //    status = (epicsThreadCreate("pollTask",
    //                                epicsThreadPriorityMedium,
    //                                epicsThreadGetStackSize(epicsThreadStackMedium),
    //                                (EPICSTHREADFUNC)pollTaskC,
    //                                this) == NULL);
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

