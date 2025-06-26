#ifndef FUNC_PROTOTYPE_USER_H
#define FUNC_PROTOTYPE_USER_H

#include <Arduino.h>
#include <FS.h>
#include <SD.h>

void MenuFuncSetDatetime(void); 
void MenuFuncCarbonCredit(void); 
void MenuFuncHistoryFault(void); 
void MenuFuncSystemInfo(void); 

// 2.System Settings
void MenuFuncSetPumpHead(void);
void MenuFuncSetMinDcv(void); 
void MenuFuncSetSlowdownDcv(void); 
void MenuFuncSetDcAmp(void);
void MenuFuncResetRuntime(void); 
void MenuFuncSetDioMode(void); 
void MenuFuncComFailAction(void); 
void MenuFuncSetVolumeUnit(void); 
void MenuFuncSetTempUnit(void); 
void MenuFuncSetTempCtrl(void); 
void MenuFuncSetOffTemp(void); 
void MenuFuncSetOnTemp(void); 
void MenuFuncSetMotorModel(void); 
void MenuFuncSetBoardSeries(void); 

// PH #n
void MenuFuncSetGearRatio(void); 
void MenuFuncSetPistonDiameter(void); 
void MenuFuncSetStrokeLength(void); 
void MenuFuncSetPumpRate(void); 
void MenuFuncCalibration(void); 

void checkTime(void);
void getVolUnit(void);
void getTempUnit(void);
double getPistonRadius(unsigned char ph);
double updatePumpRatebyStrokeRate(double stroke_rate, unsigned char ph);
double CalMaxPumpRatebyPistonLimit(unsigned char ph);
double CalMinPumpRatebyPistonLimit(unsigned char ph); 
bool ValidatePumpSetting(double v, unsigned char ph); 
void ValidateStrokeLength(double v, unsigned char ph); 
void updateStrokeCount(unsigned char pump_head);
double updateStrokeRatebyGearRatio(double gr);

//void drawMenu(MENU_CTRL_T *pctrl); //fdraw
//MENU_T *findParentMenu(MENU_T *pmenu); //ffindp
//MENU_T *findNextMenu(MENU_T *pmenu, MENU_CTRL_T *pctrl, u8 dir); //ffindm
//void MenuGetMaxValidItemsInLayer(MENU_CTRL_T *pctrl); //fcalitems
//void MenuSetItemState(MENU_CTRL_T *pctrl, u32 id, bool state) ; //fsetitems
//void menuInit(MENU_T *pmenu, MENU_CTRL_T *pctrl); //fminit
//void menuScan(MENU_T *pmenu, MENU_CTRL_T *pctrl, char key); //fmscan
//void MenuDrawArrowAtArray(const u8 array[10][2], u8 maxitem, u8 sel_item);
//void MenuMoveItemIndex(u8 *p, u8 maxitem);

// 4. SCADA Settings
void MenuFuncSetProtocol(void);
void MenuFuncSetServerID(void);
void MenuFuncSetTimeout(void);
void MenuFuncSetCoilAddr(void);
void MenuFuncSetDiscInputAddr(void);
void MenuFuncSetInputRegAddr(void);
void MenuFuncSetHoldRegAddr(void);
void MenuFuncSetBaudrate(void);
void MenuFuncSetDataBit(void);
void MenuFuncSetStopBit(void);
void MenuFuncSetParity(void);
void MenuFuncSetPumpRate(void);
void MenuFuncSetStrokeRate(void);

void mbServerRegisterWorkers(void);
void LcdUpdate(void);
void LcdPrepareBuffer(void);
void lockKeys(byte lock);
void runSdDatalogger(fs::FS &fs);
				 
#endif
