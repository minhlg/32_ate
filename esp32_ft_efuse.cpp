#include "../csiclib/csicAppLib.h"
#include  <iomanip>
#include  <iostream>
#include  <fstream>
#include  <stdlib.h>

#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Tester.h"
#include "CscDmdApi.h"
#include "CErrorException.h"
#include "CscDmd.h"
#include "CSCMultiWaveApiAll.h"

#include "esp8266_ate_test_check.h"

int csicTestNo;
int DC_scan_result = 0,AC_scan_result =0;
double DVDD_testV_1[4];
double VDD_RTC_testV_1[4];
double DVDD_testV_2[4];
double VDD_RTC_testV_2[4];
double LightSleep_IDD_VBAT[4];
double LightSleep_IDD_DVDD_IO[4];
double LS_WakeUp_IDD_VBAT[4];
double LS_WakeUp_IDD_DVDD_IO[4];
double DeepSleep_IDD_VBAT[4];
double DeepSleep_IDD_DVDD_IO[4];
double DS_WakeUp_IDD_VBAT[4];
double DS_WakeUp_IDD_DVDD_IO[4];
double Chip_PD_IDD_VBAT[4];
double Chip_PD_IDD_DVDD_IO[4];
double DynamicIDD_VBAT[4];
double DynamicIDD_DVDD_IO[4];

extern int machine_site;

//=======================
//define ate efuse struct
//=======================

struct ate_efuse {

unsigned char efuse_w_disable[2]={0x10,0};
unsigned char efuse_ate_wr_disable=1;
unsigned char wifi_mac[6]={0,0,0,0,0,0};
unsigned char mac_crc=0;
unsigned char chip_verson_repeat4=0;
unsigned char chip_ver_spiconfig_reserve[3]={0,0,0};
unsigned char ck8m_freq[2]={0,0};
unsigned char sdio_ref[2]={0,0};
unsigned char spi_pad_config[5]={0,0,0,0,0};
};

struct id_file_info{

int get_site;
char filepath[64];
unsigned long long chip_id_array[10];
int id_delta;
};


void efuse_test(int reset_test_res[4],double efuse_burned[4]);

unsigned char calcrc_1byte(unsigned char abyte)
{
   unsigned char i,crc_1byte;
   crc_1byte=0;
   for(i = 0; i < 8; i++)
    {
      if(((crc_1byte^abyte)&0x01))
         {
           crc_1byte^=0x18;
           crc_1byte>>=1;
           crc_1byte|=0x80;
          }
       else
          crc_1byte>>=1;
       abyte>>=1;
     }
     return crc_1byte;
}

unsigned char calcrc_bytes(unsigned char *p,unsigned char len)
{
 unsigned char crc=0;
 while(len--)
  {
    crc=calcrc_1byte(crc^*p++);
  }
 return crc;
}

void os_test()
{

    cscExec->Test()->setCurrentName("OS_test");
// 	cscExec->Test()->setCurrentNumber(csicTestNo);
    csicDisableDpin("LNA");
    csicSetPpmuOff("LNA");
    csicDisableDpin("TOUT");
    csicSetPpmuOff("TOUT");
    csicSetDps ("DVDD_IO", 0.0, 300 MA, CONNECT);
    csicSetDps ("VBAT", 0.0, 300 MA, CONNECT);
    csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    csicFixtureRelayOn("K1");
    csicFixtureRelayOn("K2");
    cscUtil->wait(1e-3);
    csicSetDpinLevels("allpins", 0.0, 2.5, 1.2, 1.2);
	cscTester->DCLevels()->Signal("allpins")->setForceLo();
    cscUtil->wait(1e-3);

    //csicPpmuSerialFITV ("os_neg_test", "os_open_fail", "gnd_os_pins", -10 UA, -0.9, -0.2, 1 MS,PINPMU_ADC);
    OS_Test_ESP ("os_neg_test", "os_fail", "gnd_os_pins", -10 UA, -0.9, -0.2, 1 MS,PINPMU_ADC);
    //csicPpmuSerialFITV ("os_pos_test", "os_fail", "vdd_os_pins",  100 UA, 0.2,  0.9, 1 MS,PINPMU_ADC);
    cscUtil->wait(1e-3);
    csicSetDps ("alldps",0.0, 100 MA,CONNECT);
    csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
	cscTester->DCLevels()->Signal("allpins")->setForceLo();
    //csicDisableDpin("allpins");
    //csicSetPpmuOff("allpins");
    cscUtil->wait(3e-3);

    csicSetPpmuOff("VDD");
    csicDisableDpin("VDD");

	csicFixtureRelayOff("K1");
	csicFixtureRelayOff("K2");
    //csicSetDps ("alldps",0.0, 100 MA,DISCONNECT);
    csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    cscUtil->wait(1e-3);


   return;

}


void func_measure()
{
    csicSetDpinLevels("allpins", 0.0, 2.5, 1.0, 1.8);
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	//cscTester->DCLevels()->Signal("CHIP_EN")->setForceLo();
    csicSetDps ("DVDD_IO", 2.5 , 300 MA, CONNECT);
    csicSetDps ("VBAT", 2.5, 300 MA, CONNECT);
    cscUtil->wait(5e-3);
    //csicDisableDpin ("allpins");
    csicSetDpinLevels("func_measure_allpins",0, 2.5, 1, 1.8);
    csicEnableDpin("func_measure_allpins");
    csicSetDpinLevels("XTAL_P",0, 2.5, 1, 1.8);
    csicEnableDpin("XTAL_P");
    cscUtil->wait(1e-3);
    int pause_time =0;
//	csicFuncTest("digital measure","func_dig_fail","func_measure");



    cscTester->Seq()->start("func_measure");
    while (cscTester->Seq()->getRunStatus() == BUSY) {
      if (cscTester->Seq()->isPaused()) {
         if (pause_time == 0) {
             csicDpsMeasI ("VBAT", 1 mA, Chip_PD_IDD_VBAT, 0.1 MS, 20);
             csicDpsMeasI ("DVDD_IO", 1 mA, Chip_PD_IDD_DVDD_IO, 0.1 MS, 20);

         } else if (pause_time ==1) {
             csicDpsMeasI ("VBAT", 10 mA, LightSleep_IDD_VBAT, 0.1 MS, 20);
             csicDpsMeasI ("DVDD_IO", 10 mA, LightSleep_IDD_DVDD_IO, 0.1 MS, 20);

         } else if (pause_time ==2) {
             csicDpsMeasI ("VBAT", 50 mA, LS_WakeUp_IDD_VBAT, 0.1 MS, 20);
             csicDpsMeasI ("DVDD_IO", 50 mA, LS_WakeUp_IDD_DVDD_IO, 0.1 MS, 20);

         } else if (pause_time == 3) {
             //csicPpmuFIMV ("DVDD", 0 MA,DPIN96_VRANGE_8V, DVDD_testV_1, 0.1 mS,0.1 V,1.6 V);
             //csicPpmuFIMV ("VDD_RTC", 0 MA,DPIN96_VRANGE_8V, VDD_RTC_testV_1, 0.1 mS,0.1 V,1.6 V );


             csicDpsMeasI ("VBAT", 150 mA, DynamicIDD_VBAT, 0.1 MS, 20);
             csicDpsMeasI ("DVDD_IO", 50 mA, DynamicIDD_DVDD_IO, 0.1 MS, 20);


         } else if (pause_time == 4) {
             csicDpsMeasI ("VBAT", 1 mA, DeepSleep_IDD_VBAT, 0.1 MS, 20);
             csicDpsMeasI ("DVDD_IO", 1 mA, DeepSleep_IDD_DVDD_IO, 0.1 MS, 20);


             cscUtil->wait(10e-3);

         } else if (pause_time ==5) {
             csicDpsMeasI ("VBAT", 50 mA, DS_WakeUp_IDD_VBAT, 0.1 MS, 20);
             csicDpsMeasI ("DVDD_IO", 50 mA, DS_WakeUp_IDD_DVDD_IO, 0.1 MS, 20);

         }
         pause_time++;
         cscTester->Seq()->resume();
      }
    }

             csicVariableTest("Chip_PD_IDD_VBAT","func_idd_fail",Chip_PD_IDD_VBAT,-0.5 mA,0.5 mA,"A");
             csicVariableTest("Chip_PD_IDD_DVDD_IO","func_idd_fail",Chip_PD_IDD_DVDD_IO,-0.5 mA,0.5 mA,"A");
             csicVariableTest("LightSleep_IDD_VBAT","func_idd_fail",LightSleep_IDD_VBAT,0 mA,0.5 mA,"A");
             csicVariableTest("LightSleep_IDD_DVDD_IO","func_idd_fail",LightSleep_IDD_DVDD_IO,0 mA,1 mA ,"A");
             csicVariableTest("LS_WakeUp_IDD_VBAT","func_idd_fail",LS_WakeUp_IDD_VBAT,16 mA,27 mA,"A");
             csicVariableTest("LS_WakeUp_IDD_DVDD_IO","func_idd_fail",LS_WakeUp_IDD_DVDD_IO,10 mA,18 mA,"A");
             //csicVariableTest("DVDD_V_Test","func_idd_fail",DVDD_testV_1,1.05 ,1.2 ,"V");
             //csicVariableTest("VDD_RTC_V_Test","func_idd_fail",VDD_RTC_testV_1,1.05 ,1.2 ,"V");
             csicVariableTest("DynamicIDD_VBAT","func_idd_fail",DynamicIDD_VBAT,75 mA,105 mA,"A");
             csicVariableTest("DynamicIDD_DVDD_IO","func_idd_fail",DynamicIDD_DVDD_IO,9 mA,20 mA,"A");
             csicVariableTest("DeepSleep_IDD_VBAT","func_idd_fail",DeepSleep_IDD_VBAT,-0.5 mA,0.5 mA,"A");
             csicVariableTest("DeepSleep_IDD_DVDD_IO","func_idd_fail",DeepSleep_IDD_DVDD_IO,-0.5 mA,0.5 mA,"A");
             csicVariableTest("DS_WakeUp_IDD_VBAT","func_idd_fail",DS_WakeUp_IDD_VBAT,18 mA,25 mA,"A");
             csicVariableTest("DS_WakeUp_IDD_DVDD_IO","func_idd_fail",DS_WakeUp_IDD_DVDD_IO,9 mA,16 mA,"A");

/////////////////////////////////////////////////////////

	//csicDisableDpin("func_measure_allpins");
    csicDisableDpin("XTAL_P");
	cscUtil->wait(0.001);

	//csicSetDps ("alldps", 0, 100 MA, CONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    //cscUtil->wait(5e-3);

    //csicSetDps ("alldps", 0, 100 MA, DISCONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    //cscUtil->wait(1e-3);

    return;
}
void func_ana()
{

    csicSetDpinLevels("allpins", 0.0, 2.5, 1.0, 1.8);
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	//cscTester->DCLevels()->Signal("CHIP_EN")->setForceLo();
    csicSetDps ("DVDD_IO", 2.5 , 300 MA, CONNECT);
    csicSetDps ("VBAT", 2.5, 300 MA, CONNECT);
    cscUtil->wait(5e-3);
    //csicDisableDpin ("allpins");
    csicSetDpinLevels("func_ana1_allpins",0, 2.5, 1, 1.8);
    csicEnableDpin("func_ana1_allpins");
    csicSetDpinLevels("XTAL_P",0.0, 2.5, 1.0, 1.8);
    csicEnableDpin("XTAL_P");
    cscUtil->wait(1e-3);

   csicFuncTest("digital func test","func_dig_fail","func_ana_part1");

   vector<string> lablist;
   lablist.clear();
   lablist.push_back("func_ana_part2.Capdata");
   cscTester->DCM()->add("ADC_dcm_ana",lablist,lablist,"SD_OUT");
   cscTester->DCM()->enable("ADC_dcm_ana");
   csicEnableDpin("func_ana_allpins");
   cscUtil->wait(0.001);
   cscExec->Test()->resetCumulativeResult();



    csicFuncSeq("func_ana_part2");




//    csicFuncTest("Func_ana_test","func_ana_fail","func_ana");
    while(cscTester->Seq()->getRunStatus() == BUSY);
    int capture_count = cscTester->DCM()->getCapturedCycleCount();
    if (capture_count != 1038) {
        cscExec->Datalog()->printf("capture cycle fail\ncycle is %d\n",capture_count);
    }
    int i=0;


CSC_SERIAL_BLOCK_BEGIN
    unsigned long* capture_data = new unsigned long[capture_count];
    memset(capture_data, 0, sizeof(capture_data));
    cscTester->DCM()->getCapturedDataBySite(*cscIter,"SD_OUT",0, capture_count,capture_data, DCM_HW);

    unsigned char data_to_be_checked[519];
    memset(data_to_be_checked, 0, sizeof(data_to_be_checked));
    for (i = 0; i < capture_count; i=i+2) {
        data_to_be_checked[i/2] = capture_data[i]<<4 | capture_data[i+1];
    }


    bool result = 1;

    result  = ate_check((unsigned char*)data_to_be_checked,*cscIter);

    long result_long;
    if (result) {
//        cscExec->Datalog()->printf("ate_check FAIL \n");
          result_long = 0;
    } else {
//        cscExec->Datalog()->printf("ate_check SUCCESS \n");
          result_long = 1;
    }

    csicVariableTest("Func_Ana_Test","func_ana_fail",result_long,1,1,"");
//    csicVariableTest("Func_Ana_Test","func_ana_fail",result_long,-100,100,"");

//    for (i=0; i<capture_count; i++)
//      {
//        cscExec->Datalog()->printf("%X",capture_data[i]);
//        if (i%2 == 1) { cscExec->Datalog()->printf("\n");
//        }
//
//    }
   if(capture_data!=NULL) delete [] capture_data;


CSC_SERIAL_BLOCK_END

	//csicDisableDpin("func_ana_allpins");
	cscUtil->wait(0.001);

	//csicSetDps ("alldps", 0, 100 MA, CONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    //cscUtil->wait(5e-3);

    //csicSetDps ("alldps", 0, 100 MA, DISCONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    //cscUtil->wait(1e-3);

    return;
}

void io_mbist_test()
{
   unsigned long mbist_res0[4];
   unsigned long mbist_res1[4];
   unsigned long mbist_res2[4];
   double        mbist_res[4];
   int           mbist_pass[4]={0,0,0,0};
   double        efuse_burned[4]={0,0,0,0};

    csicSetDpinLevels("allpins", 0.0, 2.5, 1.0, 1.8);
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	cscTester->DCLevels()->Signal("CHIP_EN")->setForceLo();
    csicSetDps ("DVDD_IO", 0 , 300 MA, CONNECT);
    csicSetDps ("VBAT", 0, 300 MA, CONNECT);
    cscUtil->wait(5e-3);
    //csicDisableDpin ("allpins");
    csicSetDpinLevels("io_test_allpins",0, 2.5, 1.0, 1.8);
    csicEnableDpin("io_test_allpins");
    csicSetDpinLevels("XTAL_P",0, 2.5, 1.0, 1.8);
    csicEnableDpin("XTAL_P");
    cscUtil->wait(1e-3);

    csicFuncSeq("warm_up");
    csicSetDps ("DVDD_IO", 2.5 , 300 MA, CONNECT);
    csicSetDps ("VBAT", 2.5, 300 MA, CONNECT);
    cscUtil->wait(2e-3);
	csicFuncTest("Io_test","io_fail","io_test");



//    csicFuncSeq("mbist_test");
//    csicFuncTest("mbist_test","mbist_fail","mbist_test");
    csicFuncTest("mbist_part0_conf","mbist_fail","mbist_part0_conf");
    vector<string> lablist0_mbist;
    lablist0_mbist.clear();
    lablist0_mbist.push_back("mbist_part0_cap.Capdata0_mbist");
    cscTester->DCM()->add("ADC_dcm_mbist_part0",lablist0_mbist,lablist0_mbist,"MBIST_OUT");
    cscTester->DCM()->enable("ADC_dcm_mbist_part0");
    cscUtil->wait(0.001);
    cscExec->Test()->resetCumulativeResult();
    csicFuncSeq("mbist_part0_cap");

    while(cscTester->Seq()->getRunStatus() == BUSY);
    int mbist_part0_capture_count = cscTester->DCM()->getCapturedCycleCount();
    if (mbist_part0_capture_count != 1) {
         cscExec->Datalog()->printf("mbist part0 capture cycle fail\ncycle is %d\n",mbist_part0_capture_count);
    }
CSC_SERIAL_BLOCK_BEGIN
    unsigned long* mbist_part0_capture_data = new unsigned long[mbist_part0_capture_count];
    memset(mbist_part0_capture_data, 0, sizeof(mbist_part0_capture_data));
    cscTester->DCM()->getCapturedDataBySite(*cscIter,"MBIST_OUT",0, mbist_part0_capture_count,mbist_part0_capture_data, DCM_HW);
    mbist_res0[*cscIter] = mbist_part0_capture_data[0];
    /*
    if ((mbist_res0&0x2) == 0) {
       int reset_res = 1;
       csicVariableTest("reset_clk_test_part0","reset_clk_fail",reset_res,0,0,"");
    } else if (mbist_res0 == 0x2) {
       int mbist_res = 0;
       csicVariableTest("mbist_test_part0","mbist_fail",mbist_res,-1,1,"");
    } else {
       int mbist_res = 2;
       csicVariableTest("mbist_test_part0","mbist_fail",mbist_res,-1,1,"");
    }
    */
CSC_SERIAL_BLOCK_END

    csicFuncTest("mbist_part1_conf","mbist_fail","mbist_part1_conf");
    vector<string> lablist1_mbist;
    lablist1_mbist.clear();
    lablist1_mbist.push_back("mbist_part1_cap.Capdata1_mbist");
    cscTester->DCM()->add("ADC_dcm_mbist_part1",lablist1_mbist,lablist1_mbist,"MBIST_OUT");
    cscTester->DCM()->enable("ADC_dcm_mbist_part1");
    cscUtil->wait(0.001);
    cscExec->Test()->resetCumulativeResult();
    csicFuncSeq("mbist_part1_cap");

    while(cscTester->Seq()->getRunStatus() == BUSY);
    int mbist_part1_capture_count = cscTester->DCM()->getCapturedCycleCount();
    if (mbist_part1_capture_count != 1) {
         cscExec->Datalog()->printf("mbist part1 capture cycle fail\ncycle is %d\n",mbist_part1_capture_count);
    }
CSC_SERIAL_BLOCK_BEGIN
    unsigned long* mbist_part1_capture_data = new unsigned long[mbist_part1_capture_count];
    memset(mbist_part1_capture_data, 0, sizeof(mbist_part1_capture_data));
    cscTester->DCM()->getCapturedDataBySite(*cscIter,"MBIST_OUT",0, mbist_part1_capture_count,mbist_part1_capture_data, DCM_HW);
    mbist_res1[*cscIter] = mbist_part1_capture_data[0];
    /*
    if ((mbist_res1&0x2) == 0) {
       int reset_res = 1;
       csicVariableTest("reset_clk_test_part1","reset_clk_fail",reset_res,0,0,"");
    } else if (mbist_res1 == 0x2) {
       int mbist_res = 0;
       csicVariableTest("mbist_test_part1","mbist_fail",mbist_res,-1,1,"");
    } else {
       int mbist_res = 2;
       csicVariableTest("mbist_test_part1","mbist_fail",mbist_res,-1,1,"");
    }
    */
CSC_SERIAL_BLOCK_END

    vector<string> lablist2_mbist;
    lablist2_mbist.clear();
    lablist2_mbist.push_back("mbist_part2_cap.Capdata2_mbist");
    cscTester->DCM()->add("ADC_dcm_mbist_part2",lablist2_mbist,lablist2_mbist,"MBIST_OUT");
    cscTester->DCM()->enable("ADC_dcm_mbist_part2");
    cscUtil->wait(0.001);
    cscExec->Test()->resetCumulativeResult();
    csicFuncSeq("mbist_part2_cap");

    while(cscTester->Seq()->getRunStatus() == BUSY);
    int mbist_part2_capture_count = cscTester->DCM()->getCapturedCycleCount();
    if (mbist_part2_capture_count != 1) {
         cscExec->Datalog()->printf("mbist part2 capture cycle fail\ncycle is %d\n",mbist_part2_capture_count);
    }
CSC_SERIAL_BLOCK_BEGIN
    unsigned long* mbist_part2_capture_data = new unsigned long[mbist_part2_capture_count];
    memset(mbist_part2_capture_data, 0, sizeof(mbist_part2_capture_data));
    cscTester->DCM()->getCapturedDataBySite(*cscIter,"MBIST_OUT",0, mbist_part2_capture_count,mbist_part2_capture_data, DCM_HW);

    mbist_res2[*cscIter] = mbist_part2_capture_data[0];

    //cscExec->Datalog()->printf("part2 value %d \n",mbist_part2_capture_data[0]);

    unsigned long mbist_cap0 = mbist_res0[*cscIter];
    unsigned long mbist_cap1 = mbist_res1[*cscIter];
    unsigned long mbist_cap2 = mbist_res2[*cscIter];

    //0: clk fail
    //1: mbist fail
    //2: reset fail
    //3: mbist pass

    if (((mbist_cap0&0x2) == 0) && ((mbist_cap1&0x2) == 0)) {
       mbist_res[*cscIter] = 0;
       mbist_pass[*cscIter] = 0;
    } else if (((mbist_cap0&0x2) == 2) && ((mbist_cap1&0x2) == 2) && ((mbist_cap2&0x2) == 0)) {
       mbist_res[*cscIter] = 2;
       mbist_pass[*cscIter] = 0;
    } else if((mbist_cap0 == 2) && (mbist_cap1 == 2)) {
       mbist_res[*cscIter] = 3;
       mbist_pass[*cscIter] = 1;
    } else {
       mbist_res[*cscIter] = 1;
       mbist_pass[*cscIter] = 0;
    }

CSC_SERIAL_BLOCK_END


    //csicDisableDpin("mbist_test_allpins");
    csicDisableDpin("XTAL_P");
/*
CSC_SERIAL_BLOCK_BEGIN
    unsigned long* mbist_capture_data = new unsigned long[mbist_capture_count];
    memset(mbist_capture_data, 0, sizeof(mbist_capture_data));
    cscTester->DCM()->getCapturedDataBySite(*cscIter,"MBIST_OUT",0, mbist_capture_count,mbist_capture_data, DCM_HW);
    unsigned long mbist_res0 = mbist_capture_data[0];
    unsigned long mbist_res1 = mbist_capture_data[28868];
//    cscExec->Datalog()->printf("mbist_res0 : %d,mbist_res1 : %d\n",mbist_res0,mbist_res1);
    if (((mbist_res0&0x2) == 0) & ((mbist_res1&0x2) == 0)) {
       int reset_res = 1;
       csicVariableTest("reset_clk_test","reset_clk_fail",reset_res,0,0,"");
    } else if ((mbist_res0 == 0x2) & (mbist_res1 == 0x2)) {
       int mbist_res = 0;
       csicVariableTest("mbist_test","mbist_fail",mbist_res,-1,1,"");
    } else {
       int mbist_res = 2;
       csicVariableTest("mbist_test","mbist_fail",mbist_res,-1,1,"");
    }
CSC_SERIAL_BLOCK_END
*/
    //csicDisableDpin("mbist_test_allpins");
	cscUtil->wait(0.001);

	//csicSetDps ("alldps", 0, 100 MA, CONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    //cscUtil->wait(5e-3);
    //csicSetDps ("alldps", 0, 100 MA, DISCONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    //cscUtil->wait(1e-3);

    efuse_test(mbist_pass,efuse_burned);
    csicVariableTest("clk_test","clk_fail",mbist_res,1,3,"");
    csicVariableTest("mbist_test","mbist_fail",mbist_res,2,3,"");

    //5:reset_fir_fail
    //4:reset_sec_fail
CSC_SERIAL_BLOCK_BEGIN
    if ((mbist_res[*cscIter] == 2) && (efuse_burned[*cscIter] == 1)) {
       mbist_res[*cscIter] = 4;
    } else if ((mbist_res[*cscIter] == 2) && (efuse_burned[*cscIter] == 0)) {
       mbist_res[*cscIter] = 5;
    }
CSC_SERIAL_BLOCK_END
    csicVariableTest("reset_test","reset_fir_fail",mbist_res,3,4,"");
    csicVariableTest("reset_test","reset_sec_fail",mbist_res,3,3,"");

    return;
}

void scan_test()
{
    csicSetDpinLevels("allpins", 0, 2.5, 1.0, 1.8);
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	//cscTester->DCLevels()->Signal("CHIP_EN")->setForceLo();
    csicSetDps ("DVDD_IO", 2.5 , 300 MA, CONNECT);
    csicSetDps ("VBAT",    2.5,  300 MA, CONNECT);
//    csicSetDps ("VDD11",   1.15, 300 MA, CONNECT);
    cscUtil->wait(5e-3);
    //csicDisableDpin("allpins");
    csicSetDpinLevels("dc_scan_allpins", 0, 2.5, 1.0, 1.8);
    csicSetDpinLevels("XTAL_P",0, 2.5, 1.0, 1.8);
    csicEnableDpin("dc_scan_allpins");
    //csicEnableDpin("XTAL_P");
    cscUtil->wait(1e-3);
    csicFuncSeq("wait_reset");
    csicSetDps ("DVDD11",  1.15, 100 MA, CONNECT);    //DVDD_1.1V
    csicSetDps ("VDD11",   1.15, 100 MA, CONNECT); //VDD_RTC
    cscUtil->wait(3e-3);
    csicFuncTest("Dc_Scan_test","dc_scan_fail","dc_scan");
    csicFuncTest("Ac_Scan_test","ac_scan_fail","ac_scan");
    cscTester->DPS()->Signal("DVDD11")->disconnect();
    cscTester->DPS()->Signal("VDD11")->disconnect();
    cscUtil->wait(10e-3);
    //csicFuncSeq("chip_pd");

//	csicFuncTest("Io_Bist_test","io_bist_fail","io_mbist");//just for debug

/*
    cscExec->Test()->setCurrentName("Dc_Scan_test");
    cscTester->Seq()->setStartAddress("dc_scan");
    cscUtil->wait(0.1e-3);
    cscExec->Test()->func();
    if(cscExec->Test()->getCumulativeResult() == FAIL)
       { DC_scan_result = 1;
       }
    cscExec->Test()->resetCumulativeResult();


    cscExec->Test()->setCurrentName("Ac_Scan_test");
    cscTester->Seq()->setStartAddress("ac_scan");
    cscUtil->wait(0.1e-3);
    cscExec->Test()->func();
    if(cscExec->Test()->getCumulativeResult() == FAIL)
       { AC_scan_result = 1;
       }
    cscExec->Test()->resetCumulativeResult();


*/
    //csicDisableDpin("dc_scan_allpins");
	//cscUtil->wait(0.005);

	//csicSetDps ("alldps", 0, 100 MA, CONNECT);
    //cscUtil->wait(5e-3);
    //csicSetDps ("alldps", 0, 100 MA, DISCONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    //cscUtil->wait(1e-3);
    return;

}



struct id_file_info read_chip_id(int machine_site){

struct id_file_info id_file_read;

id_file_read.get_site=machine_site;
sprintf(id_file_read.filepath,"/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP32_%d.txt",id_file_read.get_site);


if(access(id_file_read.filepath, W_OK) != 0) {
    cerr<<"Chip_ID file can not be writen! please reset as writable!\n";
    exit(1);
}

ifstream inIDFile;

inIDFile.open(id_file_read.filepath,ios::in);

if(!inIDFile){

  cerr<<"Chip id file can't be opened!0\n";
  exit(1);
}

    int inID_num = 0;

    while (inIDFile>>hex>>id_file_read.chip_id_array[inID_num]) {
    	inID_num = inID_num + 1;
      if (inID_num > 9) {
        cerr<<"Chip_ID file abnormal!1\n";
        exit(1);
      }
    }

    if (inID_num != 9) {
        cerr<<"Chip_ID file abnormal!2\n";
        exit(1);
    }
    //if ((machine_site>3) | (machine_site<1)) {
    //	printf("Please make sure you are in the engineer mode!\n\r ");
    //}
    if (id_file_read.chip_id_array[0] != machine_site) {
        cerr<<"Chip_ID file is not match with this D10!\n";
        exit(1);
    }

	if ((id_file_read.chip_id_array[8] <= id_file_read.chip_id_array[2]) | (id_file_read.chip_id_array[8] >= id_file_read.chip_id_array[3])) {
	   cerr<<"All Chip_ID has been used!\n";
       exit(1);
	}

	id_file_read.id_delta = id_file_read.chip_id_array[5] - id_file_read.chip_id_array[4];
	//printf("delta value: %d \n",delta);

	if (id_file_read.id_delta != 2) {
	   cerr<<"Chip_ID file abnormal!3\n";
       exit(1);
	}

    for (inID_num=4;inID_num<8;inID_num++) {
      if ((id_file_read.chip_id_array[inID_num+1] - id_file_read.chip_id_array[inID_num]) != id_file_read.id_delta) {
        cerr<<"Chip_ID file abnormal!4\n";
        exit(1);
      }
    }
    inIDFile.close();


    return id_file_read;

}

void update_chip_id(struct id_file_info id_file_read,unsigned long long last_id_burned){


ofstream outIDFile;
outIDFile.open(id_file_read.filepath,ios::out);

if(!outIDFile) {
        cerr<<"Chip ID file can't be opened\n";
        exit(1);
    }

int i;
for(i=4;i<9;i++){
    id_file_read.chip_id_array[i]=id_file_read.chip_id_array[i]+id_file_read.id_delta;
}

if(id_file_read.chip_id_array[8]!=last_id_burned){

        cerr<<"Chip ID can't match record!\n";
        exit(1);

}

for(i=0;i<10;i++){
    outIDFile<<hex<<id_file_read.chip_id_array[i]<<endl;
}
outIDFile.close();

}


void efuse_test_32(){

// for stdf record chip_id_value
    unsigned long long chip_id_value[4] = {0,0,0,0};
    double chip_id_value0[4] = {0,0,0,0};
    double chip_id_value1[4] = {0,0,0,0};
    double efuse_bad[4] = {0,0,0,0};

    struct ate_efuse efuse_read;
    struct ate_efuse efuse_to_burn;
    struct id_file_info id_file_read;

    unsigned long long to_burn_id_l;
    unsigned long long to_burn_id_h;




//initial chip
// read chip efuse,compare(get efuse_read)
// if not burned, burn efuse

CSC_SERIAL_BLOCK_BEGIN
   // 1,read id file,get chip id info
   // 2,make efuse_to_burn
   // 3,burn efuse
   // 4,update id file


// 1,read id file,get chip id info
id_file_read=read_chip_id(machine_site);



// 2,make efuse_to_burn
to_burn_id_l=id_file_read.chip_id_array[8]+id_file_read.id_delta;
to_burn_id_h=id_file_read.chip_id_array[1];

efuse_to_burn.wifi_mac_l[0]=to_burn_id_l%256;
efuse_to_burn.wifi_mac_l[1]=(to_burn_id_l>>8£©%256;
efuse_to_burn.wifi_mac_l[2]=(to_burn_id_l>>16£©%256;

efuse_to_burn.wifi_mac_h[0]=to_burn_id_h%256;
efuse_to_burn.wifi_mac_h[1]=(to_burn_id_h>>8£©%256;
efuse_to_burn.wifi_mac_h[2]=(to_burn_id_h>>16£©%256;

efuse_to_burn.mac_crc=calcrc_bytes(efuse_to_burn.wifi_mac,6);


// 3,burn efuse



// 4,update id file

update_chip_id(id_file_read,to_burn_id_l);

}


void efuse_test(int reset_test_res[4],double efuse_burned[4])
{
    unsigned long long chip_id_value[4] = {0,0,0,0};
    double chip_id_value0[4] = {0,0,0,0};
    double chip_id_value1[4] = {0,0,0,0};
    double efuse_bad[4] = {0,0,0,0};


    csicSetDpinLevels("allpins",0.0, 2.5, 1.0, 1.8);
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	//cscTester->DCLevels()->Signal("CHIP_EN")->setForceLo();
    csicSetDps ("DVDD_IO", 2.6 , 300 MA, CONNECT);
    csicSetDps ("VBAT",    2.6, 300 MA, CONNECT);
    cscUtil->wait(5e-3);

    //csicDisableDpin ("allpins");
//    csicSetPpmuFV("CHIP_EN",0,20 MA);
//	csicSetPpmuFV("RSTB",2.5 ,20 MA);
//	cscUtil->wait(.005);
    csicSetDpinLevels("efuse_allpins",0.0, 2.5, 1.0, 1.8);
    csicSetDpinLevels("XTAL_P",0.0, 2.5, 1.0, 1.8);
    csicEnableDpin("efuse_allpins");
    csicEnableDpin("XTAL_P");
    cscUtil->wait(1e-3);

    vector<string> lablist0;
    lablist0.clear();
    lablist0.push_back("efuse_read.Capdata0");
    vector<string> lablist1;
    lablist1.clear();
    lablist1.push_back("efuse_read.Capdata1");
    cscTester->DCM()->add("ADC_dcm_efuse",lablist0,lablist1,"EFUSE_OUT");
    cscTester->DCM()->enable("ADC_dcm_efuse");
    csicEnableDpin("efuse_allpins");
    cscUtil->wait(0.001);
    cscExec->Test()->resetCumulativeResult();

    csicFuncSeq("efuse_read");

    while(cscTester->Seq()->getRunStatus() == BUSY);
    int efuse_capture_count = cscTester->DCM()->getCapturedCycleCount();
    if (efuse_capture_count != 257) {
         cscExec->Datalog()->printf("efuse capture cycle fail\ncycle is %d\n",efuse_capture_count);
     }
    csicDisableDpin("efuse_allpins");
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	cscTester->DCLevels()->Signal("CHIP_EN")->setForceHi();
    csicDisableDpin("XTAL_P");
	cscUtil->wait(0.001);

	//csicSetDps ("alldps", 0, 100 MA, CONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    //cscUtil->wait(5e-3);
    //csicSetDps ("alldps", 0, 100 MA, DISCONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    //cscUtil->wait(1e-3);

CSC_SERIAL_BLOCK_BEGIN
    ifstream inIDFile;
    if (machine_site ==1) {
      inIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_1.txt",ios::in);
    } else if (machine_site == 2) {
    	inIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_2.txt",ios::in);
	} else if (machine_site == 3) {
    	inIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_3.txt",ios::in);
	} else if (machine_site == 4) {
    	inIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_4.txt",ios::in);
	} else if (machine_site == 5) {
    	inIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_5.txt",ios::in);
	} else if (machine_site == 6) {
    	inIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_6.txt",ios::in);
	}


    if(!inIDFile) {
        cerr<<"Chip ID file can't be opened\n";
        exit(1);
    }
    int inID_num = 0;
    unsigned long long chip_id_array[10] = {0,0,0,0,0,0,0,0,0};

    while (inIDFile>>hex>>chip_id_array[inID_num]) {
    	inID_num = inID_num + 1;
  if (inID_num > 9) {
        cerr<<"Chip_ID file abnormal!1\n";
        exit(1);
      }
    }
    if (inID_num != 9) {
        cerr<<"Chip_ID file abnormal!2\n";
        exit(1);
    }
    if ((machine_site>6) | (machine_site<1)) {
    	printf("Please make sure you are in the engineer mode!\n\r ");
    }
    else if (chip_id_array[0] != machine_site) {
        cerr<<"Chip_ID file is not match with this D10!\n";
        exit(1);
    }

	if ((chip_id_array[8] <= chip_id_array[2]) | (chip_id_array[8] >= chip_id_array[3])) {
	   cerr<<"All Chip_ID has been used!\n";
       exit(1);
	}

	int delta = chip_id_array[5] - chip_id_array[4];
	//printf("delta value: %d \n",delta);

	if ((delta != 1) & (delta != -1)) {
	   cerr<<"Chip_ID file abnormal!3\n";
       exit(1);
	}

    for (inID_num=4;inID_num<8;inID_num++) {
      if ((chip_id_array[inID_num+1] - chip_id_array[inID_num]) != delta) {
        cerr<<"Chip_ID file abnormal!4\n";
        exit(1);
      }
    }
    inIDFile.close();


    unsigned long* efuse_data_opp = new unsigned long[257];
    unsigned long* efuse_data     = new unsigned long[128];
    memset(efuse_data_opp, 1, sizeof(efuse_data_opp));
    memset(efuse_data,     1, sizeof(efuse_data));
    cscTester->DCM()->getCapturedDataBySite(*cscIter,"EFUSE_OUT",0, efuse_capture_count,efuse_data_opp, DCM_HW);
    int i =0;
    for (i=0;i<256;i++) {
      if ((i%2) == 0) {
        efuse_data[127-i/2] = efuse_data_opp[i];
//        cscExec->Datalog()->printf("num %d:%x\n",127-i/2,efuse_data[127-i/2]);
      }
    }

    unsigned long long mac_addr_read=0;
    unsigned char mac_addr_read_byte_l[4] = {0,0,0,0};
    unsigned char mac_addr_read_byte_h[3] = {0,0,0};
    unsigned char mac_crc_read_l =0;
    unsigned char mac_crc_read_h =0;
    unsigned char mac_crc_exp_l =0;
    unsigned char mac_crc_exp_h =0;

    unsigned long efuse_version_read = (efuse_data[59] <<3) + (efuse_data[58] <<2) + (efuse_data[57] <<1) + (efuse_data[56] <<0);
//    cscExec->Datalog()->printf("efuse version %d\n",efuse_version_read);
    if (efuse_version_read == 0) {
    	for (i=0;i<128;i++) {
    	  if (efuse_data[i]!=0) {
    	    efuse_bad[*cscIter] = 12;
    	  }
    	}
//        cscExec->Datalog()->printf("In case 1\n");

    } else if (efuse_version_read != 2) {
    	efuse_bad[*cscIter] = 3;
//        cscExec->Datalog()->printf("In case 1\n");
    } else {
//      cscExec->Datalog()->printf("In case 2\n");
      for (i=124;i<128;i++) {
        if (efuse_data[i] != 0) {
          efuse_bad[*cscIter] = 4;
        }
      }
      for (i=120;i<124;i++) {
        if (efuse_data[i] != 0) {
          efuse_bad[*cscIter] = 3;
        }
      }
      for (i=80;i<88;i++) {
        if (efuse_data[i] != 0) {
          efuse_bad[*cscIter] = 9;
        }
      }

      if (efuse_data[79] != 1) {
        efuse_bad[*cscIter] = 5;
      }
      if (efuse_data[78] != 0) {
        efuse_bad[*cscIter] = 6;
      }
      if (efuse_data[77] != 1) {
        efuse_bad[*cscIter] = 7;
      }
      for (i=60;i<76;i++) {
        if (efuse_data[i] != 0) {
          efuse_bad[*cscIter] = 8;
        }
      }
      for (i=0;i<16;i++) {
        if (efuse_data[i] != 0) {
          efuse_bad[*cscIter] = 2;
        }
      }
      for (i=24;i<56;i++) {
      	mac_addr_read = mac_addr_read + (efuse_data[i] << (i-24));
        mac_addr_read_byte_l[(i-24)/8] = mac_addr_read_byte_l[(i-24)/8] + (efuse_data[i] << ((i-24)%8));
      }
      for (i=16;i<24;i++) {
        mac_crc_read_l = mac_crc_read_l + (efuse_data[i] << (i-16));
      }
      mac_crc_exp_l = calcrc_bytes(mac_addr_read_byte_l,4);
//      cscExec->Datalog()->printf("efuse crc read : 0x%x,crc exp 0x%x\n",mac_crc_read,mac_crc_exp);
      if (mac_crc_exp_l != mac_crc_read_l) {
        efuse_bad[*cscIter] = 1;
      } else {
        efuse_burned[*cscIter] = 1;
      }

      if (efuse_data[76] == 1) {
        for (i=96;i<120;i++) {
          mac_addr_read_byte_h[(i-96)/8] = mac_addr_read_byte_h[(i-96)/8] + (efuse_data[i] << ((i-96)%8));
        }
        for (i=88;i<96;i++) {
          mac_crc_read_h = mac_crc_read_h + (efuse_data[i] << (i-88));
        }
        mac_crc_exp_h = calcrc_bytes(mac_addr_read_byte_h,3);
//          cscExec->Datalog()->printf("efuse crc read : 0x%x,crc exp 0x%x\n",mac_crc_read,mac_crc_exp);
        if (mac_crc_exp_h != mac_crc_read_h) {
          efuse_bad[*cscIter] = 11;
        } else {
          efuse_burned[*cscIter] = 1;
        }
      } else {
      	for (i=88;i<120;i++) {
          if (efuse_data[i] != 0) {
            efuse_bad[*cscIter] = 10;
          }
        }
      }
    }

    if (efuse_bad[*cscIter]) {
       //csicVariableTest("efuse_is_ok","efuse_fail",efuse_bad,0,0,"");
       //csicVariableTest("chip_id","efuse_fail",mac_addr_read,0,0x100000000,"");
       //csicVariableTest("efuse_burned","efuse_fail",efuse_burned,1,1,"");
       chip_id_value[*cscIter] =0;
       for (i=16;i<56;i++) {
         chip_id_value[*cscIter] = chip_id_value[*cscIter] + (efuse_data[i] << (i-16));
       }
    } else if (efuse_burned[*cscIter]) {
       //csicVariableTest("efuse_is_ok","efuse_fail",efuse_bad,0,0,"");
       //csicVariableTest("chip_id","efuse_fail",mac_addr_read,0,0x100000000,"");
       //csicVariableTest("efuse_burned","efuse_fail",efuse_burned,1,1,"");
       chip_id_value[*cscIter] = mac_addr_read;
       if ((chip_id_value[*cscIter] >= chip_id_array[3]) | (chip_id_value[*cscIter]<= chip_id_array[2])) {
         efuse_bad[*cscIter] = 12;
       }
    } else if (reset_test_res[*cscIter] == 0) {//0:mbist not pass
       chip_id_value[*cscIter] = mac_addr_read;
    } else {
    unsigned long long chip_id = 0,bit_id = 0,bit_addr_in = 0,bit_addr_out = 0;
    chip_id = chip_id_array[8];
//    cout<<"CHIP_ID is "<<chip_id+1<<endl;

    std::vector<std::string> sigGroupNames;
    std::vector<char> vec;

    int singbit = 0;
    bit_addr_in = cscTester->PatternMemory()->getLabelAddress("efuse.in_bit0");
    bit_addr_out = cscTester->PatternMemory()->getLabelAddress("efuse.out_bit80");



    chip_id = chip_id + delta;

    //printf("chip_id: %x \n",chip_id);

    chip_id_value[*cscIter]  = chip_id;
    unsigned char mac_addr_l[4];
    unsigned char mac_addr_h[3];
    unsigned char mac_addr_crc_h;
    unsigned char mac_addr_crc_l;

    mac_addr_l[0] = chip_id%256;
    mac_addr_l[1] = (chip_id>>8)%256;
    mac_addr_l[2] = (chip_id>>16)%256;
    mac_addr_l[3] = 0;
    mac_addr_h[0] = chip_id_array[1]%256;
    mac_addr_h[1] = (chip_id_array[1]>>8)%256;
    mac_addr_h[2] = (chip_id_array[1]>>16)%256;
    mac_addr_crc_l = calcrc_bytes(mac_addr_l,4);
    mac_addr_crc_h = calcrc_bytes(mac_addr_h,3);
    unsigned char efuse_version = 2;
    unsigned char efuse_data[6];
    //mac_addr_crc = 0x1;

    unsigned long* efuse_to_burn_data = new unsigned long[16];

    efuse_to_burn_data[0] = 0;
    efuse_to_burn_data[1] = 0;
    efuse_to_burn_data[2] = mac_addr_crc_l;
    efuse_to_burn_data[3] = mac_addr_l[0];
    efuse_to_burn_data[4] = mac_addr_l[1];
    efuse_to_burn_data[5] = mac_addr_l[2];
    efuse_to_burn_data[6] = mac_addr_l[3];
    efuse_to_burn_data[7] = efuse_version;
    efuse_to_burn_data[8] = 0;
    efuse_to_burn_data[9] = 0xb0;
    efuse_to_burn_data[10] = 0;
    efuse_to_burn_data[11] = mac_addr_crc_h;
    efuse_to_burn_data[12] = mac_addr_h[0];
    efuse_to_burn_data[13] = mac_addr_h[1];
    efuse_to_burn_data[14] = mac_addr_h[2];
    efuse_to_burn_data[15] = 0;


//cscExec->Datalog()->printf("\nBit num is:\n");
    for(int i=0;i<128;i++) {
        if (i%8 ==0) {
          bit_id = efuse_to_burn_data[i/8];
        }
        singbit = bit_id%2;
        bit_id = bit_id/2;
//        cscExec->Datalog()->printf("num %d:%x\n",i,singbit);
//        cout<<singbit<<endl;

        if(singbit == 1) {
            sigGroupNames.clear();sigGroupNames.push_back("SD_D1");
            vec.clear();vec.push_back('1');
            cscTester->PatternMemory()->setVectorDataAtAddress(bit_addr_in,sigGroupNames,vec);
            bit_addr_in++;
            cscTester->PatternMemory()->setVectorDataAtAddress(bit_addr_in,sigGroupNames,vec);
            bit_addr_in++;

            /*
            sigGroupNames.clear();sigGroupNames.push_back("MTDO");
            vec.clear();vec.push_back('H');
            cscTester->PatternMemory()->setVectorDataAtAddress(bit_addr_out,sigGroupNames,vec);
            bit_addr_out = bit_addr_out -2;
            */
//  cscExec->Datalog()->printf("%d ",singbit);

        }
        else
        {
            sigGroupNames.clear();sigGroupNames.push_back("SD_D1");
            vec.clear();vec.push_back('0');
            cscTester->PatternMemory()->setVectorDataAtAddress(bit_addr_in,sigGroupNames,vec);
            bit_addr_in++;
            cscTester->PatternMemory()->setVectorDataAtAddress(bit_addr_in,sigGroupNames,vec);
            bit_addr_in++;

            /*
            sigGroupNames.clear();sigGroupNames.push_back("MTDO");
            vec.clear();vec.push_back('L');
            cscTester->PatternMemory()->setVectorDataAtAddress(bit_addr_out,sigGroupNames,vec);
            bit_addr_out = bit_addr_out -2;
            */
//  cscExec->Datalog()->printf("%d ",singbit);
        }

    }
    ofstream outIDFile;
    if (machine_site == 1) {
      outIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_1.txt",ios::out);
    }	else if (machine_site == 2) {
    	outIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_2.txt",ios::out);
    }   else if (machine_site == 3) {
    	outIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_3.txt",ios::out);
    } else if (machine_site == 4) {
    	outIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_4.txt",ios::out);
    } else if (machine_site == 5) {
    	outIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_5.txt",ios::out);
    } else if (machine_site == 6) {
    	outIDFile.open("/usr/local/home/prod/DATA_LOCAL/ESP_ID_file/Chip_ID_ESP8266_6.txt",ios::out);
    }

    if(!outIDFile) {
        cerr<<"Chip ID file can't be opened\n";
        exit(1);
    }
    outIDFile<<hex<<chip_id_array[0]<<endl;
    outIDFile<<hex<<chip_id_array[1]<<endl;
    outIDFile<<hex<<chip_id_array[2]<<endl;
    outIDFile<<hex<<chip_id_array[3]<<endl;
    outIDFile<<hex<<chip_id_array[5]<<endl;
    outIDFile<<hex<<chip_id_array[6]<<endl;
    outIDFile<<hex<<chip_id_array[7]<<endl;
    outIDFile<<hex<<chip_id_array[8]<<endl;
    outIDFile<<hex<<chip_id<<endl;
    outIDFile.close();

    csicSetDpinLevels("allpins",0.0, 2.5, 1.0, 1.8);
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	//cscTester->DCLevels()->Signal("CHIP_EN")->setForceLo();
    csicSetDps ("DVDD_IO", 2.6 , 300 MA, CONNECT);
    csicSetDps ("VBAT",    2.6, 300 MA, CONNECT);
    cscUtil->wait(5e-3);

    //csicDisableDpin ("allpins");
//    csicSetPpmuFV("CHIP_EN",0,20 MA);
//	csicSetPpmuFV("RSTB",2.5 ,20 MA);
//	cscUtil->wait(.005);
    csicSetDpinLevels("efuse_allpins",0.0, 2.5, 1.0, 1.8);
    csicSetDpinLevels("XTAL_P",0.0, 2.5, 1.0, 1.8);
    csicEnableDpin("efuse_allpins");
    csicEnableDpin("XTAL_P");
    cscUtil->wait(1e-3);
    //csicFuncTest("Efuse_test","efuse_fail","efuse");
    csicFuncSeq("efuse");
//    csicVariableTest("Pause","func_idd_fail",1 ,1 ,1 ,"V");

    //csicVariableTest("chip_id","efuse_fail",chip_id,0,0x100000000,"");
    //csicVariableTest("efuse_is_ok","efuse_fail",efuse_bad,0,0,"");
    //csicVariableTest("chip_id","efuse_fail",chip_id,0,0x100000000,"");
    //csicVariableTest("efuse_burned","efuse_fail",efuse_burned,1,1,"");

    cscUtil->wait(0.001);
  }

    //csicVariableTest("efuse_is_ok","efuse_fail",efuse_bad,0,0,"");
    //csicVariableTest("chip_id","efuse_fail",chip_id,0,0x100000000,"");


    csicDisableDpin("efuse_allpins");
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	cscTester->DCLevels()->Signal("CHIP_EN")->setForceHi();
    csicDisableDpin("XTAL_P");

  chip_id_value1[*cscIter] = ((chip_id_value[*cscIter]%0x1000000)/10000000);
  chip_id_value0[*cscIter] = (chip_id_value[*cscIter]%0x1000000)-chip_id_value1[*cscIter]*10000000;


CSC_SERIAL_BLOCK_END

    csicVariableTest("chip_id_part1","efuse_fail",chip_id_value1,0,0x10000000000,"");
    csicVariableTest("chip_id_part0","efuse_fail",chip_id_value0,0,0x10000000000,"");
    csicVariableTest("efuse_is_ok","efuse_fail",efuse_bad,0,0,"");
    csicVariableTest("efuse_burned","efuse_fail",efuse_burned,0,1,"");

	//csicSetDps ("alldps", 0, 100 MA, CONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    //cscUtil->wait(5e-3);
    //csicSetDps ("alldps", 0, 100 MA, DISCONNECT);
    //csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    //csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    //cscUtil->wait(1e-3);
    return ;
}



/**********************************************************************************************************
Function Name:  user_load ()
Description:    Use to precondition the device before testing, such as control Relay power and DClevel STIL
                file.
Test Required:
Test Comment:
**********************************************************************************************************/
int user_load ()
{
  cout << "#### Test parameters setting before START testing......" << endl;

  //InitUserFlag
  cscExec->UserFlag()->setInt("csicDebug", 0);    /* Enable csicDebug*/
  cscExec->UserFlag()->setInt("csicPpmuAcc", 0);  /*Enable PPMU accuracy measurement*/
  //cscExec->Datalog()->printf("Enable test time show on datalog.\n");
  cscExec->UserFlag()->setInt("csicTimer", 0);    /* "1", Enable test time show on datalog*/
  cscExec->UserFlag()->setInt("csicStdfFlag", 0); /* "1", Enable log StdfFlag*/
  cscExec->UserFlag()->setInt("SAVE_AWAV", 0);    /* Enable csicDebug*/
  cscExec->UserFlag()->setInt("RTlogFlag", 0);    /* Enable csicDebug*/

//  cscExec->Cmd()->execute("Log Source DCM");
//  cscExec->Cmd()->execute("Log Count 10");

  //csicLoadSpecFile("myproj.spec");
  csicDibuPowerOn("P5V_RLY"); /*suggest to put this line in user_load() rather than putting in user_main()*/
  csicDibuPowerOn("P5V");
  csicFixtureRelayOff("K1");
  csicFixtureRelayOff("K2");
  return(0);
}

/**********************************************************************************************************
Function Name:  user_sot ()
Description:    Use to debug, variable setting and it be exectue when first test after loaded test program.
Test Required:
Test Comment:
**********************************************************************************************************/
int user_sot ()
{
//        csicStdfSet(1);//sampling

  csicLogTimeStart();
  static int FirstTime = 1;
  if  (FirstTime == 1)
      { cscExec->UserFlag()->setInt("FirstTime", 1);
        FirstTime = 0;}


  if  (cscExec->UserFlag()->getInt("FirstTime") == 1)
      { cout<< "#### Doing first run UserFlag setup ......" << endl;
        cscExec->Cmd()->execute("Log Source DCM");
        cscExec->Cmd()->execute("Log Count 10");
        // Do some first run need to setup
        cscExec->UserFlag()->setInt("FirstTime", 0);}

  return(0);
}

/*================= Main test program User_main () ========================================================
Function Name:  user_main ()
Description:    All test item be execute in here, you can control test item testing sequence, mask test
                item and callback subroutine etc.
Test Required:
Test Comment:
=================== End of Main test program User_main () ===============================================*/
int user_main(void)
{

    csicTestNo = 1;

    os_test();

    io_mbist_test();
    /*
    vector<unsigned long> siteList;
    vector<unsigned long>::iterator site;

    cout << "Site 0 status:" << cscExec->Sites()->getStatus(0);
    cout << endl;
    cout << "Site 1 status:" << cscExec->Sites()->getStatus(1);
    cout << endl;
    cout << "Site 2 status:" << cscExec->Sites()->getStatus(2);
    cout << endl;
    cout << "Site 3 status:" << cscExec->Sites()->getStatus(3);
    cout << endl;

    cscExec->Sites()->getEnable(siteList);
    cout << "getEnable Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;

    cscExec->Sites()->getDisable(siteList);
    cout << "getDisable Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;


    cscExec->Sites()->getActive(siteList);
    cout << "getActive Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;

    cscExec->Sites()->getEOSInactive(siteList);
    cout << "getEOSInactive Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;

*/
    //efuse_test();

    scan_test();

    func_measure();

    func_ana();

/*if (DC_scan_result==1) {
    csicBinFail("dc_scan_fail",STOP);
}

if (AC_scan_result==1) {
    csicBinFail("ac_scan_fail",STOP);
}
*/

/*
    cout << "Site 0 status:" << cscExec->Sites()->getStatus(0);
    cout << endl;
    cout << "Site 1 status:" << cscExec->Sites()->getStatus(1);
    cout << endl;
    cout << "Site 2 status:" << cscExec->Sites()->getStatus(2);
    cout << endl;
    cout << "Site 3 status:" << cscExec->Sites()->getStatus(3);
    cout << endl;

    cscExec->Sites()->getEnable(siteList);
    cout << "getEnable Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;

    cscExec->Sites()->getDisable(siteList);
    cout << "getDisable Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;


    cscExec->Sites()->getActive(siteList);
    cout << "getActive Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;

    cscExec->Sites()->getEOSInactive(siteList);
    cout << "getEOSInactive Sites: ";
    for (site = siteList.begin(); site != siteList.end(); ++site)
         cout << *site << " ";
         cout << endl;
*/
    csicBinPass("pass", CONTINUE);

	return(0);
}
/*
int user_eot(){
	cscExec->Datalog()->printf("executing user_eot(): powering down\n");

    const time_t t = std::time(NULL);
    struct tm* current_time = localtime(&t);
    cscExec->Datalog()->printf("Test end time : %d%02d%02d_%02d%02d%02d\n",1900 + current_time->tm_year, 1 + current_time->tm_mon,  current_time->tm_mday, current_time->tm_hour, current_time->tm_min, current_time->tm_sec);

    if(cscTester->Seq()->getRunStatus() == BUSY)cscTester->Seq()->forceStop();//force sequencer stop running
    csicSetPpmuOff("allpins");
	csicDisableDpin("allpins");
    csicSetDps ("VBAT", 0.0, 100 MA, CONNECT);
    csicSetDps ("DVDD_IO", 0.0, 100 MA, CONNECT);
    csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
	cscTester->DCLevels()->Signal("CHIP_EN")->setForceLo();
	CSIC_WAIT(10 MS);
    csicSetDps ("DVDD_IO", 0.0, 100 MA, DISCONNECT);
    csicSetDps ("VBAT", 0.0, 100 MA, DISCONNECT);
    csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
//	CSIC_WAIT(1000 MS);
 	return(0);
}
*/
int user_eot(){
    /*
  vector<unsigned long> siteList;
  vector<unsigned long>::iterator site;

  cout << "Site 0 status:" << cscExec->Sites()->getStatus(0);
  cout << endl;
  cout << "Site 1 status:" << cscExec->Sites()->getStatus(1);
  cout << endl;
  cout << "Site 2 status:" << cscExec->Sites()->getStatus(2);
  cout << endl;
  cout << "Site 3 status:" << cscExec->Sites()->getStatus(3);
  cout << endl;

  cscExec->Sites()->getEnable(siteList);
  cout << "getEnable Sites: ";
  for (site = siteList.begin(); site != siteList.end(); ++site)
       cout << *site << " ";
       cout << endl;

  cscExec->Sites()->getDisable(siteList);
  cout << "getDisable Sites: ";
  for (site = siteList.begin(); site != siteList.end(); ++site)
       cout << *site << " ";
       cout << endl;


  cscExec->Sites()->getActive(siteList);
  cout << "getActive Sites: ";
  for (site = siteList.begin(); site != siteList.end(); ++site)
       cout << *site << " ";
       cout << endl;

  cscExec->Sites()->getEOSInactive(siteList);
  cout << "getEOSInactive Sites: ";
  for (site = siteList.begin(); site != siteList.end(); ++site)
       cout << *site << " ";
       cout << endl;
*/
	cscExec->Datalog()->printf("executing user_eot(): powering down\n");

    const time_t t = std::time(NULL);
    struct tm* current_time = localtime(&t);
    cscExec->Datalog()->printf("Test end time : %d%02d%02d_%02d%02d%02d\n",1900 + current_time->tm_year, 1 + current_time->tm_mon,  current_time->tm_mday, current_time->tm_hour, current_time->tm_min, current_time->tm_sec);

    if(cscTester->Seq()->getRunStatus() == BUSY)cscTester->Seq()->forceStop();//force sequencer stop running
	cscTester->DCLevels()->Signal("RSTB")->setForceLo();
    CSIC_WAIT(0.5 MS);
    cscTester->DPS()->Signal("VDD11")->disconnect();
    cscTester->DPS()->Signal("DVDD11")->disconnect();
    CSIC_WAIT(20 MS);
    csicSetDps ("VBAT", 0.0, 100 MA, CONNECT);
    csicSetDps ("DVDD_IO", 0.0, 100 MA, CONNECT);
    csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    csicSetDpinLevels("allpins", 0.0, 2.5, 1.2, 1.2);
	cscTester->DCLevels()->Signal("allpins")->setForceLo();
	CSIC_WAIT(50 MS);
    csicSetDps ("DVDD_IO", 0.0, 100 MA, DISCONNECT);
    csicSetDps ("VBAT", 0.0, 100 MA, DISCONNECT);
    csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
    csicSetPpmuOff("allpins");
	csicDisableDpin("allpins");
    csicDisableDpin("LNA");
    csicSetPpmuOff("LNA");
    csicDisableDpin("TOUT");
    csicSetPpmuOff("TOUT");
	//CSIC_WAIT(500 MS);
 	return(0);
}

/*
int user_eot(){
	cscExec->Datalog()->printf("executing user_eot(): powering down\n");

    const time_t t = std::time(NULL);
    struct tm* current_time = localtime(&t);
    cscExec->Datalog()->printf("Test end time : %d%02d%02d_%02d%02d%02d\n",1900 + current_time->tm_year, 1 + current_time->tm_mon,  current_time->tm_mday, current_time->tm_hour, current_time->tm_min, current_time->tm_sec);

    if(cscTester->Seq()->getRunStatus() == BUSY)cscTester->Seq()->forceStop();//force sequencer stop running

    csicSetDps ("VBAT", 0.0, 100 MA, CONNECT);
    csicSetDps ("DVDD_IO", 0.0, 100 MA, CONNECT);
    csicSetDps ("VDD11",  0, 100 MA, CONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, CONNECT);
    CSIC_WAIT(5 MS);
	cscTester->DCLevels()->Signal("allpins")->setForceLo();
	CSIC_WAIT(15 MS);
    csicSetPpmuOff("allpins");
	csicDisableDpin("allpins");

    csicSetDps ("DVDD_IO", 0.0, 100 MA, DISCONNECT);
    csicSetDps ("VBAT", 0.0, 100 MA, DISCONNECT);
    csicSetDps ("VDD11",  0, 100 MA, DISCONNECT);
    csicSetDps ("DVDD11", 0, 100 MA, DISCONNECT);
 	return(0);
}
*/

