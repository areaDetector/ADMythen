ADMythen Releases
=======================

Code originally based on asyn driver from [LNLS] (http://lnls.cnpem.br/) 

https://epics.lnls.br/iocs/MAnyK/MAnyK_2014_02_11_01.tar.gz.

Joe Sullivan, from APS [BCDA] (http://www.aps.anl.gov/bcda/), modified the it to work with [areaDetector] (https://github.com/areaDetector).

Release Notes
============
R2-1 (December 11, 2019)
* configure/RELEAE
  - Changes to work with areaDetector R3-3.
* Driver
  - Minor improvement in debugging

R2-0 (July 4, 2017)
* Driver fixes
  - Allow compiling on Windows
  - Changes to work on big-endian machines
  - Fixed error in writeFloat64; was comparing to NUM_SD_PARAMS, should be FIRST_SD_PARAM
* Fixed medm screen to work with ADCore R3-0


1-0  (August 10, 2015)
========
* Initial release
