# ADMythen
## Introduction
This is an [EPICS](http://www.aps.anl.gov/epics/) [ADCore](https://github.com/areaDetector/ADCore) driver for [Dectris](http://www.dectris.com) Mythen strip detectors. This driver uses the socket intarface to the device and is compatible with firmware versions M3.X.X and M2.X.X.

#### Notes
* Interface to devices with the M2.X.X firmware lacks commands for retrieval of some values which means that readback values are missing.
* While the device supports per module settings the driver currently applies every setting to all modules.
* ***External trigger modes are not supported yet***

## Mythen specific parameters
| Parameter | Interface | Access | Description | Records |
|-----------|-----------|:------:|-------------|---------|
| SDSetting | asynInt32 | W | Set detector settings to one of the predefined sets.<br/>Readback value is read from the driver and not directly from device. | $(P)$(R)Setting <br/> $(P)$(R)Setting\_RBV |
| SDThreshold | asynFloat64 | RW | Energy threshold for modules. | $(P)$(R)ThresholdEnergy <br/> $(P)$(R)ThresholdEnergy\_RBV |
| SDEnergy  \* | asynFloat64 | RW | X-ray energy. | $(P)$(R)BeamEnergy <br/> $(P)$(R)BeamEnergy\_RBV |
| SDUseFlatField | asynInt32 | RW | Enable or disable flatfield correction. | $(P)$(R)UseFlatField <br/> $(P)$(R)UseFlatField\_RBV |
| SDUseCountRate | asynInt32 | RW | Enable or disable count rate correction. | $(P)$(R)UseCountRate <br/> $(P)$(R)UseCountRate\_RBV |
| SDUseBadChanInterpl | asynInt32 | RW | Enables or disables bad channel interpolation. | $(P)$(R)UseBadChanInterpl <br/> $(P)$(R)UseBadChanInterpl\_RBV |
| SDBitDepth | asynInt32 | RW | Bit length of detector pixel data. | $(P)$(R)BitDepth <br/> $(P)$(R)BitDepth\_RBV |
| SDTau | asynFloat64 | RW | Dead time constant in ns or -1 to use value from predefined settings. | $(P)$(R)Tau <br/> $(P)$(R)Tau\_RBV |
| SDReset | asynInt32 | W | Resets the detector to default settings. | $(P)$(R)Reset |
| SDFirmwareVersion | asynOctet | R | Firmware version string. | $(P)$(R)FirmwareVersion\_RBV |
| SDFirmwareMajor | asynInt32 | R | Major part of firmware number to ease hiding unsupported things in OPI. | $(P)$(R)FirmwareMajor\_RBV |
| SDNModules | asynInt32 | R | Number of modules. | $(P)$(R)NumModules\_RBV |
| SDUseGates | asynInt32 | RW\*\* | Enable or disable a gated acquisition. | $(P)$(R)UseGates <br/> $(P)$(R)UseGates\_RBV |
| SDNumGates | asynInt32 | RW\*\* | Number of gates to use if gated acquisition is enabled. | $(P)$(R)NumGates <br/> $(P)$(R)NumGates\_RBV |
\* <i>Only available on firmware M3.X.X </i><br/>
\*\* <i>Write only on firmware M2.X.X, readback is not read from the device but it's cached by the driver on writes</i>


## Standard driver parameters

Description of the changes to the standard ADDriver parameters. Only parameters with changed implementation are described here, full documentation on parameters can be found in [ADCore documentation](http://cars9.uchicago.edu/software/epics/areaDetectorDoc.html#ADDriver).

| Parameter | Description |
|-----------|-------------|
| ADImageMode | <ul> <li>**Single**:<br/>One frame is acquired from the detector and read out.</li> <li>**Multiple**:<br/>ADNumImages frames are acquired and read out from the detector.</li> <li>**Continuous**:<br/>Same as **Single** acquisition but new acquisition is automatically started as soon as one is completed. This is a software implementation of the continuous mode as hardware continuous mode is not supported.</li> </ul>|
| ADTriggerMode | <ul><li>**None**: <br/> Triggering is started as soon as a start command is recieved. </li> <li>**Single**: <br/> External trigger mode where consecutive frames are acquired in succession after a trigger signal. </li><li>**Continuous**: <br/> External trigger mode where a frame is acquired on every trigger signal. </li></ul> |
| ADAcquireTime | Acquisition time in seconds. |
| ADAcquirePeriod | Period between acquisitions in seconds. Cannot be smaller than exposure time and is automatically expanded to be at least ADAcquireTime if it is smaller. With firmwares M2.X.X the device the readback value is not read from the device but cached in the driver. |
| ADDataType | Changed to read only parameter as data type is a constant. |

#### Unsupported ADDriver parameters
Some of the standard driver parameters are not implemented for various reasons. 

| Parameter | Reason  |
|-----------|---------|
| ADTemperature, ADTemperatureActual | No temperature sensor. |
| ADGain | Detector does not use gain settings and it does not make sense to manually alter readback data. |
| ADBinX, ADBinY | Detector does not have native support for binning, driver side binning not implemented. |
| ADMinX, ADMinY | The interface only supports reading of the whole data, this would not change acquisition. ROI plugin should be used if this feature is required. |
| ADSizeX, ADSizeY | Same reason as for ADMinX, ADMinY |
| ADReverseX, ADReverseY | Not implemented. |
| ADFrameType | Not implemented. |
| ADNumExposures | See SDUseGates, SDNumGates for gating. |
| ADReadStatus | Not implemented. |
| ADTimeRemaining | **TODO** |
| Shutter related parameters | The detector does not have a shutter. |

## Build instructions
The driver depends on [ADCore](https://github.com/areaDetector/ADCore) (tested only with version R2-4) and [asyn](http://www.aps.anl.gov/epics/modules/soft/asyn/). The paths to the modules should be put into `configure/RELEASE.local`.

Example `configure/RELEASE.local`:
```
ASYN = /dls_sw/prod/R3.14.12.3/support/asyn/4-26
ADCORE = /dls_sw/prod/R3.14.12.3/support/ADCore/2-4dls6

EPICS_BASE = EPICS_BASE=/dls_sw/epics/R3.14.12.3/base
```

Example IOC provided with this distribution depends on a few other EPICS modules:
* [calc](http://www.aps.anl.gov/bcda/synApps/calc/calc.html)
* [busy](http://www.aps.anl.gov/bcda/synApps/busy/busy.html)
* [sscan](http://www.aps.anl.gov/bcda/synApps/sscan/sscan.html)
* [autosave](http://www.aps.anl.gov/bcda/synApps/autosave/autosave.html)


Example `iocs/mythenIOC/configure/RELEASE.local`:
```
ASYN = /dls_sw/prod/R3.14.12.3/support/asyn/4-26
CALC = /dls_sw/prod/R3.14.12.3/support/calc/3-1
BUSY = /dls_sw/prod/R3.14.12.3/support/busy/1-6-1
SSCAN = /dls_sw/prod/R3.14.12.3/support/sscan/2-8-1
AUTOSAVE = /dls_sw/prod/R3.14.12.3/support/autosave/5-0dls3
ADCORE = /dls_sw/prod/R3.14.12.3/support/ADCore/2-4dls6

EPICS_BASE = /dls_sw/epics/R3.14.12.3/base
```

The driver can be compiled by issuing `make`.

## Example IOC

**TODO** Write documentation on the IOC

## Operator screens

**TODO** GUIs are not yet created

### TODOs
- [ ] For now the driver always returns 32 bit unsigned integer array instead of adapting the data type to the bit depth.
- [ ] The documentation of driver is still lacking.
- [ ] External trigger modes are untested and not implemented properly

