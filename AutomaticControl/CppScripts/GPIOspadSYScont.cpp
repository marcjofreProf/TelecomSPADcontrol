/* Author: Prof. Marc Jofre
Dept. Network Engineering
Universitat Politècnica de Catalunya - Technical University of Catalonia

Modified: 2026
Created: 2025

Script for PRU real-time handling of multi SPAD system
*/
#include "GPIOspadSYScont.h"
#include<iostream>
#include<fstream>
#include<bitset>
#include<string>
#include<sstream> // For istringstream
#include<cstdlib>
#include<cstdio>
#include<fcntl.h>
#include<unistd.h>
#include<sys/epoll.h>
#include<thread>
#include<pthread.h>
#include<unistd.h>
#include <algorithm> // For std::nth_element
#include <signal.h>
// Watchdog
#include <sys/ioctl.h>
#include <linux/watchdog.h>
// Handling priority in task manager
#include<sched.h>
// Time/synchronization management
#include <chrono>
// Mathemtical calculations
#include <cmath>// abs, fmod, fmodl, floor, ceil
// PRU programming
#include<poll.h>
#include <stdio.h>
#include <sys/mman.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#define PRU_Operation_NUM 0 // PRU operation and handling with PRU0
#define PRU_Signal_NUM 1 // Signals PINS with PRU1
/******************************************************************************
* Local Macro Declarations - Global Space point of View                       *
******************************************************************************/
#define AM33XX_PRUSS_IRAM_SIZE 8192 // Instructions RAM (where .p assembler instructions are loaded)
#define AM33XX_PRUSS_DRAM_SIZE 8192 // PRU's Data RAM
#define AM33XX_PRUSS_SHAREDRAM_SIZE 12000 // Shared Data RAM

#define PRU_ADDR        0x4A300000      // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN         0x80000         // Length of PRU memory

#define DDR_BASEADDR 0x80000000 //0x80000000 is where DDR starts, but we leave some offset (0x00001000) to avoid conflicts with other critical data present// Already initiated at this position with LOCAL_DDMinit
#define OFFSET_DDR 0x00001000
#define SHAREDRAM 0x00010000 // Already initiated at this position with LOCAL_DDMinit
#define OFFSET_SHAREDRAM 0x00000000 //Global Memory Map (from the perspective of the host) equivalent with 0x00002000
#define LAST_SHAREDRAMPOS 2999 // (AM33XX_PRUSS_SHAREDRAM_SIZE)/4-1 //Num address postion to the last one
#define PRU0_DATARAM 0x00000000 //Global Memory Map (from the perspective of the host)// Already initiated at this position with LOCAL_DDMinit
#define PRU1_DATARAM 0x00002000 //Global Memory Map (from the perspective of the host)// Already initiated at this position with LOCAL_DDMinit
#define DATARAMoffset 0x00000200 // Offset from Base OWN_RAM to avoid collision with some data. // Already initiated at this position with LOCAL_DDMinit

#define PRUSS0_PRU0_DATARAM 0
#define PRUSS0_PRU1_DATARAM 1
#define PRUSS0_PRU0_IRAM 2
#define PRUSS0_PRU1_IRAM 3
#define PRUSS0_SHARED_DATARAM 4

using namespace std;

namespace exploringBB {
void* exploringBB::GPIO::ddrMem = nullptr; // Define and initialize ddrMem
void* exploringBB::GPIO::sharedMem = nullptr; // Define and initialize
void* exploringBB::GPIO::pru0dataMem = nullptr; // Define and initialize 
void* exploringBB::GPIO::pru1dataMem = nullptr; // Define and initialize
void* exploringBB::GPIO::pru_int = nullptr;// Define and initialize
unsigned int* exploringBB::GPIO::sharedMem_int = nullptr;// Define and initialize
unsigned int* exploringBB::GPIO::pru0dataMem_int = nullptr;// Define and initialize
unsigned int* exploringBB::GPIO::pru1dataMem_int = nullptr;// Define and initialize
int exploringBB::GPIO::mem_fd = -1;// Define and initialize 

GPIO::GPIO(){// Redeclaration of constructor GPIOspadSYScont when no argument is specified
	// Some variable initialization
	// Initialize structure used by prussdrv_pruintc_intc
	// PRUSS_INTC_INITDATA is found in pruss_intc_mapping.h
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	// Allocate and initialize memory
	prussdrv_init();
	// Interrupts
	if (prussdrv_open(PRU_EVTOUT_0) == -1) {// Event PRU-EVTOUT0 - Interrupt from PRU0
		perror("prussdrv_open(PRU_EVTOUT_0) failed. Execute as root: sudo su or sudo. /boot/uEnv.txt has to be properly configured with iuo. Message: "); 
	}
	
	if (prussdrv_open(PRU_EVTOUT_1) == -1) {// Event PRU-EVTOUT1 - Interrupt from PRU1
		perror("prussdrv_open(PRU_EVTOUT_1) failed. Execute as root: sudo su or sudo. /boot/uEnv.txt has to be properly configured with iuo. Message: "); 
	}
	
	// Map PRU's interrupts
	// prussdrv.pruintc_init(); // Init handling interrupts from PRUs
	prussdrv_pruintc_init(&pruss_intc_initdata);
	
	// Clear prior interrupt events
	prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
	prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
	
	
    // Initialize DDM
	LOCAL_DDMinit(); // DDR (Double Data Rate): A class of memory technology used in DRAM where data is transferred on both the rising and falling edges of the clock signal, effectively doubling the data rate without increasing the clock frequency.
	// Here we can update memory space assigned address
	valpHolder=(unsigned short*)&sharedMem_int[OFFSET_SHAREDRAM+1];
	//valpAuxHolder=valpHolder+4+6*NumQuBitsPerRun;// 6* since each detection also includes the channels (2 Bytes) and 4 bytes for 32 bits counter, and plus 4 since the first tag is captured at the very beggining
	CalpHolder=(unsigned int*)&sharedMem_int[OFFSET_SHAREDRAM];//(unsigned int*)&pru0dataMem_int[2];// First tagg captured at the very beggining
	
	// Launch the PRU0 (timetagging) and PR1 (generating signals) codes but put them in idle mode, waiting for command
	// Timetagging
	// Execute program
	// Load and execute the PRU program on the PRU0
	/*
	pru0dataMem_int[0]=static_cast<unsigned int>(0); // set no command
	pru0dataMem_int[1]=static_cast<unsigned int>(this->NumQuBitsPerRun); // set number captures, with overflow clock
	pru0dataMem_int[2]=static_cast<unsigned int>(this->GuardPeriod);// Indicate period of the sequence signal, so that it falls correctly and is picked up by the Signal PRU. Link between system clock and PRU clock. It has to be a power of 2
	pru0dataMem_int[3]=static_cast<unsigned int>(1);
	pru0dataMem_int[4]=static_cast<unsigned int>(this->TTGcoincWin); // set coincidence window length
	*/
	if (prussdrv_exec_program(PRU_Operation_NUM, "./CppScripts/BBBhw/PRUassTaggDetScriptSimple.bin") == -1){
		if (prussdrv_exec_program(PRU_Operation_NUM, "./BBBhw/PRUassTaggDetScriptSimple.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUassTaggDetScriptSimple.bin");
		}
	}
	////prussdrv_pru_enable(PRU_Operation_NUM);
	
	// Generate signals
	/*
	pru1dataMem_int[0]=static_cast<unsigned int>(0); // set no command
	pru1dataMem_int[1]=static_cast<unsigned int>(this->NumberRepetitionsSignal); // set the number of repetitions
	pru1dataMem_int[2]=static_cast<unsigned int>(1);// Referenced to the synch trig period
	pru1dataMem_int[3]=static_cast<unsigned int>(this->GuardPeriod);// Indicate period of the sequence signal, so that it falls correctly and is picked up by the Signal PRU. Link between system clock and PRU clock. It has to be a power of 2
	pru1dataMem_int[4]=static_cast<unsigned int>(this->ContCorr);
	pru1dataMem_int[5]=static_cast<unsigned int>(this->SigONPeriod);
	pru1dataMem_int[6]=static_cast<unsigned int>(this->ContCorrSign);
	pru1dataMem_int[7]=static_cast<unsigned int>(this->SigOFFPeriod);// Off time
	*/
	// Load and execute the PRU program on the PRU1
	if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/BBBhw/PRUassTrigSigScriptHist4Sig.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/BBBhw/PRUassTrigSigScript.bin") == -1){
		if (prussdrv_exec_program(PRU_Signal_NUM, "./BBBhw/PRUassTrigSigScriptHist4Sig.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./BBBhw/PRUassTrigSigScript.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUassTrigSigScriptHist4Sig.bin");//perror("prussdrv_exec_program non successfull writing of PRUassTrigSigScript.bin");
		}
	}
	/*
	// Self Test Histogram - comment the PRU1 launching above "generate signals"
	if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/BBBhw/PRUassTrigSigScriptHist4SigSelfTest.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/BBBhw/PRUassTrigSigScript.bin") == -1){
		if (prussdrv_exec_program(PRU_Signal_NUM, "./BBBhw/PRUassTrigSigScriptHist4SigSelfTest.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./BBBhw/PRUassTrigSigScript.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUassTrigSigScriptHist4SigSelfTest.bin");//perror("prussdrv_exec_program non successfull writing of PRUassTrigSigScript.bin");
		}
	}
	sleep(10);// Give some time to load programs in PRUs and initiate. Very important, otherwise bad values might be retrieved
	this->SendTriggerSignalsSelfTest(); // Self test initialization
	cout << "Attention doing SendTriggerSignalsSelfTest. To be removed" << endl;	
	*/

	////prussdrv_pru_enable(PRU_Signal_NUM);
	sleep(1); // Give some time to load programs in PRUs and the synch protocols to initiate and lock after prioritazion and adjtimex. Very important, otherwise bad values might be retrieved
	
	  /*// Doing debbuging checks - Debugging 1	  
	  std::thread threadReadTimeStampsAux=std::thread(&GPIO::ReadTimeStamps,this);
	  std::thread threadSendTriggerSignalsAux=std::thread(&GPIO::SendTriggerSignals,this);
	  threadReadTimeStampsAux.join();	
	  threadSendTriggerSignalsAux.join();
	  //this->DDRdumpdata(); // Store to file
	  
	  //munmap(ddrMem, 0x0FFFFFFF); // remove any mappings for those entire pages containing any part of the address space of the process starting at addr and continuing for len bytes. 
	  //close(mem_fd); // Device
	  streamDDRpru.close();
	  prussdrv_pru_disable(PRU_Signal_NUM);
	  prussdrv_pru_disable(PRU_Operation_NUM);  
	  prussdrv_exit();*/
	  ///////////////////////////////////////////////////////
	//this->setMaxRrPriority();// for the main instance. But it stalls operation in RealTime kernel.
	//////////////////////////////////////////////////////////
}

int GPIO::InitAgentProcess(){
	// Launch periodic synchronization of the IEP timer - like slotted time synchronization protocol
	////this->threadRefSynch=std::thread(&GPIO::PRUsignalTimerSynch,this);// More absolute in time
 	//this->threadRefSynch=std::thread(&GPIO::PRUsignalTimerSynchJitterLessInterrupt,this);// reduce the interrupt jitter of non-real-time OS
 	////this->threadRefSynch.detach();// If detach, then at the end comment the join. Otherwise, uncomment the join().
	return 0; //All OK
}
/////////////////////////////////////////////////////////
bool GPIO::setMaxRrPriority(int PriorityValAux){// For rapidly handling interrupts
	int max_priority=sched_get_priority_max(SCHED_FIFO);
	int Nice_priority=PriorityValAux;//73;// Higher priority. Very important parameter to have stability of the measurements. Slightly smaller than the priorities for clock control (ptp4l,...) but larger than for the general program
	// SCHED_RR: Round robin
	// SCHED_FIFO: First-In-First-Out
	sched_param sch_params;
	sch_params.sched_priority = Nice_priority;
	if (sched_setscheduler(0,SCHED_FIFO,&sch_params)==-1){
		cout <<" Failed to set maximum real-time priority." << endl;
		return false;
	}
	return true;
}

/// Errors handling
std::atomic<bool> signalReceivedFlag{false};
static void SignalINTHandler(int s) {
signalReceivedFlag.store(true);
cout << "Caught SIGINT" << endl;
}

static void SignalPIPEHandler(int s) {
signalReceivedFlag.store(true);
cout << "Caught SIGPIPE" << endl;
}
////////////////////////////////////////////////////////
void GPIO::acquire() {
/*while(valueSemaphore==0);
this->valueSemaphore=0; // Make sure it stays at 0
*/
// https://stackoverflow.com/questions/61493121/when-can-memory-order-acquire-or-memory-order-release-be-safely-removed-from-com
// https://medium.com/@pauljlucas/advanced-thread-safety-in-c-4cbab821356e
//int oldCount;
	//unsigned long long int ProtectionSemaphoreTrap=0;
	bool valueSemaphoreExpected=true;
	while(true){
	//oldCount = this->valueSemaphore.load(std::memory_order_acquire);
	//if (oldCount > 0 && this->valueSemaphore.compare_exchange_strong(oldCount,oldCount-1,std::memory_order_acquire)){
		//ProtectionSemaphoreTrap++;
		//if (ProtectionSemaphoreTrap>UnTrapSemaphoreValueMaxCounter){this->release();cout << "GPIO::Releasing semaphore!!!" << endl;}// Avoid trapping situations
		if (this->valueSemaphore.compare_exchange_strong(valueSemaphoreExpected,false,std::memory_order_acquire)){	
			break;
		}
	}
}

void GPIO::release() {
this->valueSemaphore.store(true,std::memory_order_release); // Make sure it stays at 1
//this->valueSemaphore.fetch_add(1,std::memory_order_release);
}
//////////////////////////////////////////////
struct timespec GPIO::SetWhileWait(){
	struct timespec requestWhileWaitAux;
	this->TimePointClockCurrentSynchPRU1future=this->TimePointClockCurrentSynchPRU1future+std::chrono::nanoseconds(this->TimePRU1synchPeriod);
	
	auto duration_since_epochFutureTimePoint=this->TimePointClockCurrentSynchPRU1future.time_since_epoch();
	// Convert duration to desired time
	long long int TimePointClockCurrentFinal_time_as_count = static_cast<long long int>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epochFutureTimePoint).count())-static_cast<long long int>(2*TimePRUcommandDelay); // Add an offset, since the final barrier is implemented with a busy wait
	//cout << "TimePointClockCurrentFinal_time_as_count: " << TimePointClockCurrentFinal_time_as_count << endl;

	requestWhileWaitAux.tv_sec=(int)(TimePointClockCurrentFinal_time_as_count/((long)1000000000));
	requestWhileWaitAux.tv_nsec=(long)(TimePointClockCurrentFinal_time_as_count%(long)1000000000);

	// Timer file descriptor sets an interrupt that if not commented (when not in use) produces a long reaction time in the while loop (busy wait)
	//if (this->iIterPRUcurrentTimerVal==0){ // Needed to configure it only at the first iteration
	// By forcing it to renew the timerfd everytime, the awakenen time is not advanced (which probably is better)
		TimePointClockCurrentFinal_time_as_count = static_cast<long long int>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epochFutureTimePoint).count());//-static_cast<long long int>(this->TimeClockMarging); // Add an offset, since the final barrier is implemented with a busy wait 
		//cout << "TimePointClockCurrentFinal_time_as_count: " << TimePointClockCurrentFinal_time_as_count << endl;
		
	    TimerTimeout.tv_sec = (int)((5*TimePRUcommandDelay)/((long)1000000000)); 
	    TimerTimeout.tv_usec = (long)((5*TimePRUcommandDelay)%(long)1000000000);

	    struct itimerspec its;
	    its.it_interval.tv_sec = (int)(static_cast<unsigned long long int>(static_cast<int>(TimePRU1synchPeriod)-duration_FinalInitialMeasTrigAuxAvg)/((long)1000000000));  // Periodic interval expiration // Make it periodic to try to be more deterministic. No interval, one-shot timer
	    its.it_interval.tv_nsec = (long)(static_cast<unsigned long long int>(static_cast<int>(TimePRU1synchPeriod)-duration_FinalInitialMeasTrigAuxAvg)%(long)1000000000);
	    its.it_value.tv_sec=(int)((TimePointClockCurrentFinal_time_as_count-static_cast<long long int>(duration_FinalInitialMeasTrigAuxAvg))/((long)1000000000)); // Initial expiration
		its.it_value.tv_nsec=(long)((TimePointClockCurrentFinal_time_as_count-static_cast<long long int>(duration_FinalInitialMeasTrigAuxAvg))%(long)1000000000);

		timerfd_settime(this->tfd, TFD_TIMER_ABSTIME, &its, NULL);

		// Watch timefd file descriptor
	    FD_ZERO(&rfds);
	    FD_SET(this->tfd, &rfds);
	//}

	return requestWhileWaitAux;
}

int GPIO::ReadTimeStamps(int iIterRunsAux,int QuadEmitDetecSelecAux, double SynchTrigPeriodAux,unsigned int NumQuBitsPerRunAux, double* FineSynchAdjValAux, unsigned long long int QPLAFutureTimePointNumber, bool QPLAFlagTestSynchAux){// Read the detected timestaps in four channels
/////////////
	/*
	try{
	this->QPLAFlagTestSynch=QPLAFlagTestSynchAux;
	SynchTrigPeriod=SynchTrigPeriodAux;// Histogram/Period value
	NumQuBitsPerRun=NumQuBitsPerRunAux;
	//cout << "GPIO::ReadTimeStamps NumQuBitsPerRun: " << NumQuBitsPerRun << endl;
	//valpAuxHolder=valpHolder+4+6*NumQuBitsPerRun;// 6* since each detection also includes the channels (2 Bytes) and 4 bytes for 32 bits counter, and plus 4 since the first tag is captured at the very beggining
	AccumulatedErrorDriftAux=FineSynchAdjValAux[0];// Synch trig offset
	AccumulatedErrorDrift=FineSynchAdjValAux[1]; // Synch trig frequency
	QuadEmitDetecSelecGPIO=QuadEmitDetecSelecAux;// Update value
	std::chrono::nanoseconds duration_back(QPLAFutureTimePointNumber);
	this->QPLAFutureTimePoint=Clock::time_point(duration_back);
	this->QPLAFutureTimePoint=this->QPLAFutureTimePoint-std::chrono::nanoseconds(static_cast<unsigned long long int>(2.0*GuardPeriod*PRUclockStepPeriodNanoseconds));// Timetagger starts listening 2 periods in advance to avoid interrupt and signals to arrive concurrently at the timetagger
	requestSemaphoreWhileWait=SemaphoreSetWhileWait();
	this->QPLAFutureTimePoint=this->QPLAFutureTimePoint+std::chrono::nanoseconds(7*TimePRUcommandDelay);// Give some margin so that ReadTimeStamps and coincide in the respective methods of GPIO. Only for th einitial run, since the TimeStaps are run once (to enter the acquire in GPIO). What consumes time is writting to PRU, then times 4 since 4 writings to PRU before sleep in GPIO
	this->QPLAFutureTimePointSleep=this->QPLAFutureTimePoint;// Update value
	this->QPLAFutureTimePoint=this->QPLAFutureTimePoint+std::chrono::nanoseconds(1*TimePRUcommandDelay);// Crucial to make the link between PRU clock and system clock (already well synchronized). Two memory mapping to PRU
	SynchRem=static_cast<int>((static_cast<long double>(1.5*GuardPeriod)-fmodl(static_cast<long double>(PRUoffsetDriftErrorAbsAvg)+(static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(QPLAFutureTimePoint.time_since_epoch()).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds)),static_cast<long double>(GuardPeriod)))*static_cast<long double>(PRUclockStepPeriodNanoseconds));// For time stamping it waits 1.5
	this->QPLAFutureTimePoint=this->QPLAFutureTimePoint+std::chrono::nanoseconds(SynchRem);
	clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&requestSemaphoreWhileWait,NULL); // Synch barrier. so the time within acquired semaphore is not so large
	//cout << "Before this->ManualSemaphore...to be commented" << endl;
	while (this->ManualSemaphore and whileProtAux>0){whileProtAux--;};// Wait other process// Very critical to not produce measurement deviations when assessing the periodic snchronization
	whileProtAux=whileProtAuxMax;
	//cout << "After this->ManualSemaphore...to be commented" << endl;
	this->ManualSemaphoreExtra=true;
	this->ManualSemaphore=true;// Very critical to not produce measurement deviations when assessing the periodic snchronization
	this->acquire();// Very critical to not produce measurement deviations when assessing the periodic snchronization
	this->AdjPulseSynchCoeffAverage=static_cast<long double>(this->EstimateSynchAvg);// Acquire this value for the this tag reading set
	///////////
	if (QPLAFlagTestSynchAux==true){TagsSeparationDetRelFreqAdpSlope=TagsSeparationDetRelFreqAdpSlopeSynch;}
	else{TagsSeparationDetRelFreqAdpSlope=TagsSeparationDetRelFreqAdpSlopeRegular;}
	if (this->GuardPeriod<=(this->MultFactorEffSynchPeriod*this->SynchTrigPeriod)){cout << "GPIO::ReadTimeStamps Attention!!! GuardPeriod smaller or equal than the effective period...check inconsistency!!!" << endl;}
	pru0dataMem_int[2]=static_cast<unsigned int>(this->GuardPeriod);// Indicate guard period of the sequence signal, so that it falls correctly and it is picked up by the Signal PRU. Link between system clock and PRU clock. It has to be a power of 2
	pru0dataMem_int[1]=static_cast<unsigned int>(this->NumQuBitsPerRun); // set number captures
	// Different modes of periodic correction
	
	// Smart version of the truncation - avoid being at the border of transition
	if (SynchPlaneDomainMode==true){
		if (abs(PRUoffsetDriftErrorAbsAvg-PRUoffsetDriftErrorAbsAvgOldTruncatedRecv)>truncatedSynchTrigPeriod){
			truncatedPRUoffsetDriftErrorAbsAvg=round(PRUoffsetDriftErrorAbsAvg/truncatedSynchTrigPeriod)*truncatedSynchTrigPeriod;
			TimePointUpdateFlagAux=true;
		}
		else{
			truncatedPRUoffsetDriftErrorAbsAvg=truncatedPRUoffsetDriftErrorAbsAvgOldRecv;
		}
		PRUoffsetDriftErrorAbsAvgOldTruncatedRecv=PRUoffsetDriftErrorAbsAvg;// Update value
		truncatedPRUoffsetDriftErrorAbsAvgOldRecv=truncatedPRUoffsetDriftErrorAbsAvg; // Update value
	}
	else{ // Real-time clock dominates
		truncatedPRUoffsetDriftErrorAbsAvg=0.0;
	}
	
	// The time in PRU units to consider (as an approximation) for correction with relative frequency correction is composed of half the effective guard period due to interrupt alignment handling, the effective period, the time since last emission detection, then again MultFactorEffSynchPeriod*SynchTrigPeriod more or less
	// Correcting for relative frequency difference is an approximation game (due to all the variable involved). The best is to have all hardware clocks so in-phase synchronized that there is no relative frequency difference.
	//ldTimePointClockTagPRUDiff=static_cast<long double>(0.5*MultFactorEffSynchPeriod*SynchTrigPeriod)+static_cast<long double>(0.5*GuardPeriod)+0.5*NumSynchMeasAvgAux*static_cast<long double>(TimePRU1synchPeriod)/static_cast<long double>(PRUclockStepPeriodNanoseconds);//static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint-this->QPLAFutureTimePointOld).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);// update value
	// Since we truncated the offset value (due to the jitter) it no longer makes sense that the QPLAFutureTimePointOld is used. Instead, it makes more sense, to take the last time the offset exceeded the truncation
	//ldTimePointClockTagPRUDiff=static_cast<long double>(0.5*MultFactorEffSynchPeriod*SynchTrigPeriod)+static_cast<long double>(0.5*GuardPeriod)+static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint-this->QPLAFutureTimePointReadTimeStampsOld).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);// update value
	ldTimePointClockTagPRUDiff=static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint.time_since_epoch()).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);// update value
	
	if (TimePointUpdateFlagAux==true){
		this->QPLAFutureTimePointReadTimeStampsOld=this->QPLAFutureTimePoint;
		TimePointUpdateFlagAux=false;
	}

	switch (SynchCorrectionTimeFreqNoneFlag){
		case 3:{// Time and frequency correction			
			PRUoffFreqTotalAux=static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg)+ldTimePointClockTagPRUDiff*static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg);
			break;
		}
		case 2:{// Time correction
			PRUoffFreqTotalAux=static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg);
			break;
		}
		case 1:{ // Frequency correction
			PRUoffFreqTotalAux=ldTimePointClockTagPRUDiff*static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg);
			break;
		}
		default:{PRUoffFreqTotalAux=0.0;break;}// None time nor frequency correction
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Now, The time in PRU units to consider (as an approximation) for correction with relative frequency correction is composed of the effective period, and the period
	if (abs(AccumulatedErrorDrift)<AccumulatedErrorDriftThresh){AccumulatedErrorDrift=0.0;}// Do not apply computed synch relative frequency difference
	//cout << "GPIO::ReadTimeStamps AccumulatedErrorDrift: " << AccumulatedErrorDrift << endl;
	switch (SynchCorrectionTimeFreqNoneFlag){
		case 3:{// Time and frequency correction			
			PRUoffFreqTotalAux+=static_cast<long double>(AccumulatedErrorDriftAux)+static_cast<long double>(AccumulatedErrorDrift)*ldTimePointClockTagPRUDiff;
			break;
		}
		case 2:{// Time correction
			PRUoffFreqTotalAux+=static_cast<long double>(AccumulatedErrorDriftAux);
			break;
		}
		case 1:{ // Frequency correction
			PRUoffFreqTotalAux+=static_cast<long double>(AccumulatedErrorDrift)*ldTimePointClockTagPRUDiff;
			break;
		}
		default:{PRUoffFreqTotalAux+=0.0;break;}// None time nor frequency correction
	}
	
	// Attention ldTimePointClockTagPRUinitial is needed in DDRdumpdata!!!!
	ldTimePointClockTagPRUinitial=static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod)+static_cast<long double>(GuardPeriod)+static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint.time_since_epoch()).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);;//+static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint.time_since_epoch()).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);//+static_cast<long double>(AccumulatedErrorDriftAux)+static_cast<long double>(0.5*MultFactorEffSynchPeriod*SynchTrigPeriod);// Time point after all PRU synch steps. The periodic synch offset is not accounted for
	
	// Accounting for the effective offset and frequency correction
	if (PRUoffFreqTotalAux<0.0){
		PRUoffFreqTotalAux=-fmodl(-PRUoffFreqTotalAux,static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod));
	}
	else{
		PRUoffFreqTotalAux=fmodl(PRUoffFreqTotalAux,static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod));
	}
	// Prepare the overall correction for PRU loops (so divide by 2)
	PRUoffFreqTotalAux=(static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod)+PRUoffFreqTotalAux)/2.0;
	if (PRUoffFreqTotalAux<=0.0){PRUoffFreqTotalAux=1.0;}// To avoid having PRU stall

	pru0dataMem_int[3]=static_cast<unsigned int>(PRUoffFreqTotalAux);

	// Different time tagging window size, when synching with respect normal operation.
	if (QPLAFlagTestSynchAux==true){pru0dataMem_int[4]=static_cast<unsigned int>(this->TTGcoincWin);}// Minimum width for the timetagging coincidence window, to have accuracy
	else{pru0dataMem_int[4]=static_cast<unsigned int>(this->TTGcoincWin);} // set coincidence window length

	// Sleep barrier to synchronize the different nodes at this point, so the below calculations and entry times coincide
	requestCoincidenceWhileWait=CoincidenceSetWhileWait();
	clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&requestCoincidenceWhileWait,NULL); // Synch barrier. So that SendTriggerSignals and ReadTimeStamps of the different nodes coincide	
	pru0dataMem_int[0]=static_cast<unsigned int>(QuadEmitDetecSelecAux); // set command
	//QPLAFutureTimePoint=QPLAFutureTimePoint-std::chrono::nanoseconds(duration_FinalInitialMeasTrigAuxAvg);// Actually, the time measured duration_FinalInitialMeasTrigAuxAvg is not indicative of much (only if it changes a lot to high values it means trouble)
	// Set top priority
	//this->setMaxRrPriority(PriorityValTop);
	while (Clock::now()<QPLAFutureTimePoint);// Busy wait time synch sending signals. With while loop, it is more aggresive to take control (hence fall within the correct interrupt period) but has more variation (but it does not matter since there will be the proceedure to synch in the PRU)
	prussdrv_pru_send_event(21);
	//this->TimePointClockTagPRUfinal=Clock::now();// Compensate for delays
	// Set regular priority
	//this->setMaxRrPriority(PriorityValRegular);
	//retInterruptsPRU0=prussdrv_pru_wait_event_timeout(PRU_EVTOUT_0,WaitTimeInterruptPRU0);// First interrupt sent to measure time
	//  PRU long execution making sure that notification interrupts do not overlap
	retInterruptsPRU0=prussdrv_pru_wait_event_timeout(PRU_EVTOUT_0,WaitTimeInterruptPRU0);

	//cout << "retInterruptsPRU0: " << retInterruptsPRU0 << endl;
	if (retInterruptsPRU0>0){
		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);// So it has time to clear the interrupt for the later iterations
	}
	else if (retInterruptsPRU0==0){
		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);// So it has time to clear the interrupt for the later iterations
		cout << "GPIO::ReadTimeStamps took to much time for the TimeTagg. Timetags might be inaccurate. Reset PRUO if necessary." << endl;		
	}
	else{
		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);// So it has time to clear the interrupt for the later iterations
		cout << "PRU0 interrupt poll error" << endl;
	}

	// Freeing the semaphore is done in the proper line inside DDRdumpdata
	//this->ManualSemaphore=false;
	//this->ManualSemaphoreExtra=false;
	//this->release();

	// Debugging
	//cout << "GPIO::ReadTimeStamps QuadEmitDetecSelecGPIO: " << QuadEmitDetecSelecGPIO << endl;

	this->DDRdumpdata(iIterRunsAux); // Pre-process tags. Needs to access memory of PRU, so better within the controlled acquired environment

	}
      catch (const std::exception& e) {
	// Handle the exception
      	this->release();
      	cout << "GPIO::ReadTimeStamps Exception: " << e.what() << endl;
     }
    */
return 0;// all ok
}

int GPIO::SendTriggerSignals(int QuadEmitDetecSelecAux, double SynchTrigPeriodAux,unsigned int NumberRepetitionsSignalAux,double* FineSynchAdjValAux,unsigned long long int QPLAFutureTimePointNumber, bool QPLAFlagTestSynchAux){ // Uses output pins to clock subsystems physically generating qubits or entangled qubits
	/*
	try{
	this->QPLAFlagTestSynch=QPLAFlagTestSynchAux;
	SynchTrigPeriod=SynchTrigPeriodAux;// Histogram/Period value
	NumberRepetitionsSignal=static_cast<unsigned int>(NumberRepetitionsSignalAux);// Number of repetitions to send signals
	AccumulatedErrorDriftAux=FineSynchAdjValAux[0];// Synch trig offset
	AccumulatedErrorDrift=FineSynchAdjValAux[1]; // Synch trig frequency
	QuadEmitDetecSelecGPIO=QuadEmitDetecSelecAux;// Update value
	std::chrono::nanoseconds duration_back(QPLAFutureTimePointNumber);
	this->QPLAFutureTimePoint=Clock::time_point(duration_back);
	requestSemaphoreWhileWait=SemaphoreSetWhileWait();
	this->QPLAFutureTimePoint=this->QPLAFutureTimePoint+std::chrono::nanoseconds(7*TimePRUcommandDelay);// Give some margin so that ReadTimeStamps and coincide in the respective methods of GPIO. Only for th einitial run, since the TimeStaps are run once (to enter the acquire in GPIO). What consumes time is writting to PRU, then times 4 since 4 writings to PRU before sleep in GPIO
	this->QPLAFutureTimePointSleep=this->QPLAFutureTimePoint;// Update value
	this->QPLAFutureTimePoint=this->QPLAFutureTimePoint+std::chrono::nanoseconds(1*TimePRUcommandDelay);// Since one memory mapping to PRU memory
	//SynchRem=static_cast<int>((static_cast<long double>(1.5*GuardPeriod)-fmodl((static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint.time_since_epoch()).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds)),static_cast<long double>(GuardPeriod)))*static_cast<long double>(PRUclockStepPeriodNanoseconds));
	SynchRem=static_cast<int>((static_cast<long double>(1.5*GuardPeriod)-fmodl(static_cast<long double>(PRUoffsetDriftErrorAbsAvg)+(static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint.time_since_epoch()).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds)),static_cast<long double>(GuardPeriod)))*static_cast<long double>(PRUclockStepPeriodNanoseconds));
	this->QPLAFutureTimePoint=this->QPLAFutureTimePoint+std::chrono::nanoseconds(SynchRem);
	clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&requestSemaphoreWhileWait,NULL); // Synch barrier. so the time within acquired semaphore is not so large
	while (this->ManualSemaphore and whileProtAux>0){whileProtAux--;};// Wait other process// Very critical to not produce measurement deviations when assessing the periodic snchronization
	whileProtAux=whileProtAuxMax;
	this->ManualSemaphoreExtra=true;
	this->ManualSemaphore=true;// Very critical to not produce measurement deviations when assessing the periodic snchronization
	this->acquire();// Very critical to not produce measurement deviations when assessing the periodic snchronization
	//this->ManualSemaphore=true;// Very critical to not produce measurement deviations when assessing the periodic snchronization
	// Apply a slotted synch configuration (like synchronized Ethernet)
	this->AdjPulseSynchCoeffAverage=static_cast<long double>(this->EstimateSynchAvg);
	// Different Signal ON time, when synching with respect normal operation.
	if (QPLAFlagTestSynchAux==true){pru1dataMem_int[5]=static_cast<unsigned int>(minSigONPeriod);this->SigOFFPeriod=this->SynchTrigPeriod-minSigONPeriod;}// Minimum width time on, to have accuracy
	else{pru1dataMem_int[5]=static_cast<unsigned int>(this->SigONPeriod);this->SigOFFPeriod=this->SynchTrigPeriod-this->SigONPeriod;} // signal width time on in regular operation
	if (SigOFFPeriod<1.0){SigOFFPeriod=1.0; cout << "GPIO::SendTriggerSignals SigOFFPeriod smaller than 1 PRU unit...check inconsistency!!!" << endl;}
	if (this->GuardPeriod<=(this->MultFactorEffSynchPeriod*this->SynchTrigPeriod)){cout << "GPIO::SendTriggerSignals Attention!!! GuardPeriod smaller or equal than the effective period...check inconsistency!!!" << endl;}
	pru1dataMem_int[7]=static_cast<unsigned int>(this->SigOFFPeriod);// Off time
	pru1dataMem_int[3]=static_cast<unsigned int>(this->GuardPeriod);// Indicate period of the sequence signal, so that it falls correctly and is picked up by the Signal PRU. Link between system clock and PRU clock. It has to be a power of 2
	pru1dataMem_int[1]=static_cast<unsigned int>(this->NumberRepetitionsSignal); // set the number of repetitions
	// Different modes of periodic correction
	//switch (QuadEmitDetecSelecGPIO){
	//	case 1: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePointOld1;break;}
	//	case 2: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePointOld2;break;}
	//	case 3: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePointOld3;break;}
	//	case 4: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePointOld4;break;}
	//	case 5: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePointOld5;break;}
	//	case 6: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePointOld6;break;}
	//	case 7: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePointOld7;break;}
	//	default: {break;}
	//}
	
	// There is an intrinsic limitation estimating PRUoffsetDriftErrorAbsAvg, which is impaired by the time jitter of handling the interrupt to check the curren tPRU clock.
	// At some point, it could be that the PRU clock is more stable than this jitter (even more so if SyncE synchronizaed the clocks).
	// Then, for the triggering of the signals, it is better to truncate this value to a sub multiple of the SynchTrigPeriod; and let the qubits handle the time offset synchronization from this point onwards
	// The qubits continuously correct for the time offset difference due to the QPLA function (SmallDriftContinuousCorrection) that after each masurements send to the sender the correction estimated to the time offfseet
	// With godd enough hardware clocks, the jitter is limited by the BBB kernel determinisms, in the order of 5us. With hundreds of averaging it is reduced this jitter around 10 times (since it scales as sqrt(NumberAveraging))
	// Therefore, the truncation could be a submultiple of SynchTrigPeriod which is slightly larger than the averaged interrupt jitter (around 500ns, which in PRU units would be 100)
	// Better to be a number multiple of power of 2 and larger than this minim jitter PRU value
	//if (PRUoffsetDriftErrorAbsAvg<0.0){
	//	truncatedPRUoffsetDriftErrorAbsAvg=-round((-PRUoffsetDriftErrorAbsAvg+truncatedSynchTrigPeriod/2.0)/truncatedSynchTrigPeriod)*truncatedSynchTrigPeriod;
	//}
	//else{
	//	truncatedPRUoffsetDriftErrorAbsAvg=round((PRUoffsetDriftErrorAbsAvg+truncatedSynchTrigPeriod/2.0)/truncatedSynchTrigPeriod)*truncatedSynchTrigPeriod;
	//}
	if (SynchPlaneDomainMode==true){
		// Smart version of the truncation - avoid being at the border of transition
		if (abs(PRUoffsetDriftErrorAbsAvg-PRUoffsetDriftErrorAbsAvgOldTruncatedEmit)>truncatedSynchTrigPeriod){
			truncatedPRUoffsetDriftErrorAbsAvg=round(PRUoffsetDriftErrorAbsAvg/truncatedSynchTrigPeriod)*truncatedSynchTrigPeriod;
			TimePointUpdateFlagAux=true;
		}
		else{
			truncatedPRUoffsetDriftErrorAbsAvg=truncatedPRUoffsetDriftErrorAbsAvgOldEmit;
		}
		PRUoffsetDriftErrorAbsAvgOldTruncatedEmit=PRUoffsetDriftErrorAbsAvg;// Update value
		truncatedPRUoffsetDriftErrorAbsAvgOldEmit=truncatedPRUoffsetDriftErrorAbsAvg; // Update value
	}
	else{// Real-time clock dominates
		truncatedPRUoffsetDriftErrorAbsAvg=0.0;
	}
		
	// The time in PRU units to consider (as an approximation) for correction with relative frequency correction is composed of half the effective period due to interrupt alignment handling, the effective period, the time since last emission detection, then again MultFactorEffSynchPeriod*SynchTrigPeriod more or less
	//ldTimePointClockTagPRUDiff=static_cast<long double>(0.5*MultFactorEffSynchPeriod*SynchTrigPeriod)+static_cast<long double>(0.5*GuardPeriod)+0.5*NumSynchMeasAvgAux*static_cast<long double>(TimePRU1synchPeriod)/static_cast<long double>(PRUclockStepPeriodNanoseconds);//static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint-this->QPLAFutureTimePointOld).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);// update value
	// Since we truncated the offset value (due to the jitter) it no longer makes sense that the QPLAFutureTimePointOld is used. Instead, it makes more sense, to take the last time the offset exceeded the truncation
	//ldTimePointClockTagPRUDiff=static_cast<long double>(0.5*MultFactorEffSynchPeriod*SynchTrigPeriod)+static_cast<long double>(0.5*GuardPeriod)+static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint-this->QPLAFutureTimePointSendTriggerSignalsOld).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);// update value
	ldTimePointClockTagPRUDiff=static_cast<long double>(std::chrono::duration_cast<std::chrono::nanoseconds>(this->QPLAFutureTimePoint.time_since_epoch()).count())/static_cast<long double>(PRUclockStepPeriodNanoseconds);// update value
	
	if(TimePointUpdateFlagAux==true){
		this->QPLAFutureTimePointSendTriggerSignalsOld=this->QPLAFutureTimePoint;
		TimePointUpdateFlagAux=false;
	}

	switch (SynchCorrectionTimeFreqNoneFlag){
		case 3:{// Time and frequency correction			
			PRUoffFreqTotalAux=static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg)+ldTimePointClockTagPRUDiff*static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg);
			break;
		}
		case 2:{// Time correction
			PRUoffFreqTotalAux=static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg);
			break;
		}
		case 1:{ // Frequency correction
			PRUoffFreqTotalAux=ldTimePointClockTagPRUDiff*static_cast<long double>(truncatedPRUoffsetDriftErrorAbsAvg);
			break;
		}
		default:{PRUoffFreqTotalAux=0.0;break;}// None time nor frequency correction
	}

	//switch (QuadEmitDetecSelecGPIO){// Update value	
	//	case 1: {this->QPLAFutureTimePointOld1=this->QPLAFutureTimePoint;break;}
	//	case 2: {this->QPLAFutureTimePointOld2=this->QPLAFutureTimePoint;break;}
	//	case 3: {this->QPLAFutureTimePointOld3=this->QPLAFutureTimePoint;break;}
	//	case 4: {this->QPLAFutureTimePointOld4=this->QPLAFutureTimePoint;break;}
	//	case 5: {this->QPLAFutureTimePointOld5=this->QPLAFutureTimePoint;break;}
	//	case 6: {this->QPLAFutureTimePointOld6=this->QPLAFutureTimePoint;break;}
	//	case 7: {this->QPLAFutureTimePointOld7=this->QPLAFutureTimePoint;break;}
	//	default: {this->QPLAFutureTimePointOld=this->QPLAFutureTimePoint;break;}
	//}

	////////////////////////////////////////////////////////////////////////////////////////////
	// The synch offset
	// Now, The time in PRU units to consider (as an approximation) for correction with relative frequency correction is composed of the effective period, and the period		
	if (abs(AccumulatedErrorDrift)<AccumulatedErrorDriftThresh){AccumulatedErrorDrift=0.0;}// Do not apply computed synch relative frequency difference
	switch (SynchCorrectionTimeFreqNoneFlag){
		case 3:{// Time and frequency correction			
			PRUoffFreqTotalAux+=static_cast<long double>(AccumulatedErrorDriftAux)+static_cast<long double>(AccumulatedErrorDrift)*ldTimePointClockTagPRUDiff;
			break;
		}
		case 2:{// Time correction
			PRUoffFreqTotalAux+=static_cast<long double>(AccumulatedErrorDriftAux);
			break;
		}
		case 1:{ // Frequency correction
			PRUoffFreqTotalAux+=static_cast<long double>(AccumulatedErrorDrift)*ldTimePointClockTagPRUDiff;
			break;
		}
		default:{PRUoffFreqTotalAux+=0.0;break;}// None time nor frequency correction
	}
		
	if (SynchCorrectionTimeFreqNoneFlag==1 or SynchCorrectionTimeFreqNoneFlag==3){ // It has to be a much finer control at PRU level (and also this level) in order to be able to compensate for the very low hardware wandering/drift.
		// Compute the number of quarter passes since it is histogram analysis of 4 symbols to add/substract 1 PRU unit of compensation
		long double AccumulatedErrorDriftPRUoffsetDriftErrorAvg=static_cast<long double>(AccumulatedErrorDrift)+PRUoffsetDriftErrorAvg;
		// First sum the adimensional relative frequency differences, due to the internal system_clock and the one wanted to be compensated from the synch algorithm.
		// Then, invert the value, since it will tell the number of PRU units so that this value produces 1 PRU unit difference (either positive of negative).
		// Then, it is divided by 4 because it is histogram analysis
		// Then, it is divided by the period length
		// Then is multiplied by 2, because the assembler code can do loops that consume double
		//cout << "GPIO::SendTriggerSignals AccumulatedErrorDriftPRUoffsetDriftErrorAvg: " << AccumulatedErrorDriftPRUoffsetDriftErrorAvg << endl;
		if (AccumulatedErrorDriftPRUoffsetDriftErrorAvg==0.0){ContCorr=static_cast<unsigned int>(4294967295);}
		else{
			long double AccumulatedErrorDriftPRUoffsetDriftErrorAvgAux=2.0*(1.0/abs(AccumulatedErrorDriftPRUoffsetDriftErrorAvg))/static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod);
			if (AccumulatedErrorDriftPRUoffsetDriftErrorAvgAux>4294967295.0){AccumulatedErrorDriftPRUoffsetDriftErrorAvgAux=4294967295.0;}
			else if (AccumulatedErrorDriftPRUoffsetDriftErrorAvgAux<1.0){AccumulatedErrorDriftPRUoffsetDriftErrorAvgAux=1.0;}
			ContCorr=static_cast<unsigned int>(AccumulatedErrorDriftPRUoffsetDriftErrorAvgAux);
		}
		if (AccumulatedErrorDriftPRUoffsetDriftErrorAvg<0.0){
			ContCorrSign=static_cast<unsigned int>(((SynchTrigPeriod-SigONPeriod)-4.0-4.0)/2.0+1.0);// 1 positive unit intra pulses relative frequency difference correction
		}
		else if(AccumulatedErrorDriftPRUoffsetDriftErrorAvg>0.0){
			ContCorrSign=static_cast<unsigned int>(((SynchTrigPeriod-SigONPeriod)-4.0-4.0)/2.0-1.0);// 1 negative unit intra pulses relative frequency difference correction
		}
		else{
			ContCorrSign=static_cast<unsigned int>(((SynchTrigPeriod-SigONPeriod)-4.0-4.0)/2.0);// No intra pulses relative frequency difference correction
		}
	}
	else{// Do not apply relative frequency correction in the PRU script
		ContCorr=static_cast<unsigned int>(4294967295);
		ContCorrSign=static_cast<unsigned int>(((SynchTrigPeriod-SigONPeriod)-4.0-4.0)/2.0);// No intra pulses relative frequency difference correction
	}

	// Accounting for the effective offset and frequency correction
	if (PRUoffFreqTotalAux<0.0){
		PRUoffFreqTotalAux=-fmodl(-PRUoffFreqTotalAux,static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod));
	}
	else{
		PRUoffFreqTotalAux=fmodl(PRUoffFreqTotalAux,static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod));
	}
	// Prepare the overall correction for PRU loops (so divide by 2)
	PRUoffFreqTotalAux=(static_cast<long double>(MultFactorEffSynchPeriod*SynchTrigPeriod)+PRUoffFreqTotalAux)/2.0;
	if (PRUoffFreqTotalAux<=0.0){PRUoffFreqTotalAux=1.0;}
	pru1dataMem_int[2]=static_cast<unsigned int>(PRUoffFreqTotalAux);
	// Particular for histogram, tell the adjusted period accounting for relative frequency difference
	pru1dataMem_int[4]=static_cast<unsigned int>(ContCorr);// Referenced to the corrected synch period
	pru1dataMem_int[6]=static_cast<unsigned int>(ContCorrSign);// Referenced to the corrected synch period

	// Sleep barrier to synchronize the different nodes at this point, so the below calculations and entry times coincide
	requestCoincidenceWhileWait=CoincidenceSetWhileWait();
	clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&requestCoincidenceWhileWait,NULL); // Synch barrier. So that SendTriggerSignals and ReadTimeStamps of the different nodes coincide	

	pru1dataMem_int[0]=static_cast<unsigned int>(QuadEmitDetecSelecAux); // set command. Generate signals. Takes around 900000 clock ticks
	//this->QPLAFutureTimePoint=this->QPLAFutureTimePoint-std::chrono::nanoseconds(duration_FinalInitialMeasTrigAuxAvg); // Actually, the time measured duration_FinalInitialMeasTrigAuxAvg is not indicative of much (only if it changes a lot to high values it means trouble)
	// Set top priority
	//this->setMaxRrPriority(PriorityValTop);
	////if (Clock::now()<this->QPLAFutureTimePoint){cout << "Check that we have enough time" << endl;}
	while (Clock::now()<this->QPLAFutureTimePoint);// Busy wait time synch sending signals. With while loop, it is more aggresive to take control (hence fall within the correct interrupt period) but has more variation (but it does not matter since there will be the proceedure to synch in the PRU)
	// Important, the following line at the very beggining to reduce the command jitter
	prussdrv_pru_send_event(22);//Send host arm to PRU1 interrupt
	//this->TimePointClockSynchPRUfinal=Clock::now();
	// Set regular priority
	//this->setMaxRrPriority(PriorityValRegular);
	// Here there should be the instruction command to tell PRU1 to start generating signals
	// We have to define a command, compatible with the memory space of PRU0 to tell PRU1 to initiate signals
	//  PRU long execution making sure that notification interrupts do not overlap
	retInterruptsPRU1=prussdrv_pru_wait_event_timeout(PRU_EVTOUT_1,WaitTimeInterruptPRU1);

	//cout << "SendTriggerSignals: retInterruptsPRU1: " << retInterruptsPRU1 << endl;
	if (retInterruptsPRU1>0){
		prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);// So it has time to clear the interrupt for the later iterations
	}
	else if (retInterruptsPRU1==0){
		prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);// So it has time to clear the interrupt for the later iterations
		cout << "GPIO::SendTriggerSignals took to much time. Reset PRU1 if necessary." << endl;
	}
	else{
		prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);// So it has time to clear the interrupt for the later iterations
		cout << "PRU1 interrupt error" << endl;
	}

	this->ManualSemaphore=false;
	this->ManualSemaphoreExtra=false;
	this->release();

	}
      catch (const std::exception& e) {
	// Handle the exception
      	this->release();
      	cout << "GPIO::SendTriggerSignals Exception: " << e.what() << endl;
     }
*/
return 0;// all ok	
}

int GPIO::DDRdumpdata(int iIterRunsAux){
/*//cout << "GPIO::Reading timetags" << endl;
// Reading data from PRU shared and own RAMs
//DDR_regaddr = (short unsigned int*)ddrMem + OFFSET_DDR;
valp=valpHolder; // Coincides with SHARED in PRUassTaggDetSimpleScript.p
//valpAux=valpAuxHolder;
//synchp=synchpHolder;
//for each capture bursts, at the beggining is stored the overflow counter of 32 bits. From there, each capture consists of 32 bits of the DWT_CYCCNT register and 8 bits of the channels detected (40 bits per detection tag).
// The shared memory space has 12KB=12×1024bytes=12×1024×8bits=98304bits.
//Doing numbers, we can store up to 1966 captures. To be in the safe side, we can do 1964 captures

// When unsgined char
//valThresholdResetCounts=static_cast<unsigned int>(*valpAux);
//valpAux++;// 1 times 8 bits
//valThresholdResetCounts=valThresholdResetCounts | (static_cast<unsigned int>(*valpAux))<<8;
//valpAux++;// 1 times 8 bits
//valThresholdResetCounts=valThresholdResetCounts | (static_cast<unsigned int>(*valpAux))<<16;
//valpAux++;// 1 times 8 bits
//valThresholdResetCounts=valThresholdResetCounts | (static_cast<unsigned int>(*valpAux))<<24;
//valpAux++;// 1 times 8 bits
// When unsigned short - Not used
//valThresholdResetCounts=static_cast<unsigned int>(*valpAux);
//valpAux++;// 1 times 16 bits
//valThresholdResetCounts=valThresholdResetCounts | (static_cast<unsigned int>(*valpAux))<<16;
//valpAux++;// 1 times 16 bits
//cout << "valThresholdResetCounts: " << valThresholdResetCounts << endl;
//////////////////////////////////////////////////////////////////////////////

// Reading first calibration tag and link it to the system clock
OldLastTimeTagg=static_cast<unsigned long long int>(*CalpHolder);//extendedCounterPRUaux + static_cast<unsigned long long int>(*CalpHolder);
//cout << "GPIO::OldLastTimeTagg: " << OldLastTimeTagg << endl;

// Slot the TimeTaggsLast, since it eventually has to start at the beggining of the effective period
this->TimeTaggsLast=(static_cast<unsigned long long int>(ldTimePointClockTagPRUinitial)/static_cast<unsigned long long int>(GuardPeriod)+2)*static_cast<unsigned long long int>(GuardPeriod);//(static_cast<unsigned long long int>(ldTimePointClockTagPRUinitial)/static_cast<unsigned long long int>(MultFactorEffSynchPeriod*SynchTrigPeriod))*static_cast<unsigned long long int>(MultFactorEffSynchPeriod*SynchTrigPeriod);

//Furthermore, remove some time from epoch - in multiples of the SynchTrigPeriod, so it is easier to handle in the above agents
//cout << "GPIO::DDRdumpdata TimeTaggsLast before removing Epoch: " << TimeTaggsLast << endl;
if (this->TimeTaggsLast<((this->ULLIEpochReOffset/static_cast<unsigned long long int>(GuardPeriod))*static_cast<unsigned long long int>(GuardPeriod))){
	cout << "GPIO::DDRdumpdata ULLIEpochReOffset is too large...not removing actual time since epoch!!!! check: " << endl;
}
else{
	this->TimeTaggsLast=static_cast<unsigned long long int>(static_cast<long long int>(this->TimeTaggsLast)-static_cast<long long int>((this->ULLIEpochReOffset/static_cast<unsigned long long int>(GuardPeriod))*static_cast<unsigned long long int>(GuardPeriod)));//static_cast<unsigned long long int>(static_cast<long long int>(this->TimeTaggsLast)-static_cast<long long int>((this->ULLIEpochReOffset/static_cast<unsigned long long int>(MultFactorEffSynchPeriod*SynchTrigPeriod))*static_cast<unsigned long long int>(MultFactorEffSynchPeriod*SynchTrigPeriod)));
}
//cout << "GPIO::DDRdumpdata TimeTaggsLast after removing Epoch: " << TimeTaggsLast << endl;
if (iIterRunsAux==0){TimeTaggsLastStored=TimeTaggsLast;TotalCurrentNumRecords=0;}// First iteration of current runs, store the value for synchronization time difference calibration

long long int LLIOldLastTimeTagg=static_cast<long long int>(OldLastTimeTagg);
// It is important to adjust the OldLastTimeTagg so that absolute timetagg references match among nodes
LLIOldLastTimeTagg=LLIOldLastTimeTagg%(static_cast<unsigned long long int>(MultFactorEffSynchPeriod*SynchTrigPeriod));

unsigned int valCycleCountPRUAux1;
unsigned int valCycleCountPRUAux2;
//cout << "GPIO::NumQuBitsPerRun " << NumQuBitsPerRun << endl;
//cout << "GPIO::MaxNumQuBitsMemStored " << MaxNumQuBitsMemStored << endl;
//for (iIterDump=0; iIterDump<NumQuBitsPerRun; iIterDump++){
CurrentiIterDump=0;
int CurrentiIterDumpAux=0;
bool ValidTag=false;

extendedCounterPRUholder=1;// Re-initialize at each run. 1 so that at least the first is checked and stored
extendedCounterPRUholderOld=0;// Re-initialize at each run
int TotalCurrentNumRecordsOld=TotalCurrentNumRecords;
int PendingNumQuBitsPerRun=static_cast<int>(sharedMem_int[OFFSET_SHAREDRAM+LAST_SHAREDRAMPOS]);
//cout << "GPIO::DDRdumpdata NumQuBitsPerRun-PendingNumQuBitsPerRun: " << (NumQuBitsPerRun-PendingNumQuBitsPerRun) << endl;
while (CurrentiIterDumpAux<NumQuBitsPerRun and extendedCounterPRUholder>extendedCounterPRUholderOld and (NumQuBitsPerRun-PendingNumQuBitsPerRun)>CurrentiIterDump){// Do it until a timetagg is smaller in value than the previous one, because it means that it could not achieve to capture NumQuBitsPerRun
	extendedCounterPRUholderOld=extendedCounterPRUholder;
	// When unsigned short
	//valCycleCountPRU=static_cast<unsigned int>(0);// Reset value
	valCycleCountPRUAux1=static_cast<unsigned int>(*valp) & 0x0000FFFF;
	//cout << "GPIO::DDRdumpdata::static_cast<unsigned int>(*valp): " << static_cast<unsigned int>(*valp) << endl;
	//cout << "GPIO::DDRdumpdata::valCycleCountPRUAux1: " << valCycleCountPRUAux1 << endl;
	valp++;// 1 times 16 bits
	valCycleCountPRUAux2=((static_cast<unsigned int>(*valp))<<16) & 0xFFFF0000;
	//cout << "GPIO::DDRdumpdata::static_cast<unsigned int>(*valp): " << static_cast<unsigned int>(*valp) << endl;
	//cout << "GPIO::DDRdumpdata::valCycleCountPRUAux2: " << valCycleCountPRUAux2 << endl;
	valCycleCountPRU=valCycleCountPRUAux1 | valCycleCountPRUAux2;
	valp++;// 1 times 16 bits
	//if (iIterDump==0 or iIterDump== 512 or iIterDump==1023){cout << "valCycleCountPRU: " << valCycleCountPRU << endl;}
	// Mount the extended counter value
	extendedCounterPRUholder=static_cast<unsigned long long int>(valCycleCountPRU);//extendedCounterPRUaux + static_cast<unsigned long long int>(valCycleCountPRU);
	//if (iIterDump==0 or iIterDump== 512 or iIterDump==1023){cout << "extendedCounterPRU: " << extendedCounterPRU << endl;}
	// Apply system clock corrections
	//TimeTaggsStored[TotalCurrentNumRecords]=static_cast<unsigned long long int>(static_cast<long double>(static_cast<long long int>(extendedCounterPRUholder)-LLIOldLastTimeTagg)*AdjPulseSynchCoeffAverage)+TimeTaggsLast;	// The fist OldLastTimeTagg and TimeTaggsLast of the iteration is compensated for with the calibration tag together with the accumulated synchronization error	    
	TimeTaggsStored[TotalCurrentNumRecords]=static_cast<unsigned long long int>(static_cast<long long int>(extendedCounterPRUholder)-LLIOldLastTimeTagg)+TimeTaggsLast;	// The fist OldLastTimeTagg and TimeTaggsLast of the iteration is compensated for with the calibration tag together with the accumulated synchronization error	    
	//////////////////////////////////////////////////////////////		
	// When unsigned short
	valp++;// 1 times 16 bits
	// Check that it belong to a channel of interest
	//cout << "GPIO::ChannelTagsStored[TotalCurrentNumRecords]: " << ChannelTagsStored[TotalCurrentNumRecords] << endl;
	//cout << "GPIO::ValidTagMask: " << ValidTagMask << endl;
	if ((ChannelTagsStored[TotalCurrentNumRecords]&ValidTagMask)>0){ValidTag=true;}
	else{ValidTag=false;}
	//cout << "GPIO::ValidTag: " << ValidTag << endl;
	//cout << "GPIO::TotalCurrentNumRecords: " << TotalCurrentNumRecords << endl;
	//cout << "GPIO::extendedCounterPRUholder: " << extendedCounterPRUholder << endl;
	//cout << "GPIO::extendedCounterPRUholder>0: " << (extendedCounterPRUholder>0) << endl;
	if (TotalCurrentNumRecords<MaxNumQuBitsMemStored and extendedCounterPRUholder>0 and ValidTag==true){TotalCurrentNumRecords++;CurrentiIterDump++;}//Variable to hold the number of currently stored records in memory	
	CurrentiIterDumpAux++; //Safety variable
}
if (TotalCurrentNumRecords>MaxNumQuBitsMemStored){cout << "GPIO::We have reached the maximum number of qubits storage!" << endl;}
else if (TotalCurrentNumRecords==TotalCurrentNumRecordsOld){cout << "GPIO::No detection of qubits!" << endl;}
//cout << "GPIO::TotalCurrentNumRecords: " << TotalCurrentNumRecords << endl;
//cout << "GPIO::Clearing PRU timetags" << endl;
//// Reset values of the sharedMem_int after each iteration
//for (iIterDump=0; iIterDump<((NumQuBitsPerRun/2)*3); iIterDump++){
//	sharedMem_int[OFFSET_SHAREDRAM+iIterDump]=static_cast<unsigned int>(0x00000000); // Put it all to zeros
//}
// Actually only needed to zero the first memory position
sharedMem_int[OFFSET_SHAREDRAM+1]=static_cast<unsigned int>(0x00000000); // Put it all to zeros

// Freeeing the semaphore block after all the access to the PRU shared memory
this->ManualSemaphore=false;
this->ManualSemaphoreExtra=false;
this->release();
*/
return 0; // all ok
}

int GPIO::IntMeanFilterSubArray(int* ArrayHolderAux,int MeanFilterFactor){
	if (MeanFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
		int temp=0.0;
		for(int i = 0; i < MeanFilterFactor; i++) {
			temp += ArrayHolderAux[i];
		}
		
		return temp/static_cast<int>(MeanFilterFactor);
	}
}

int GPIO::IntMedianFilterSubArray(int* ArrayHolderAux,int MedianFilterFactor){
	if (MedianFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
		/*
		// Non-efficient code
		// Step 1: Copy the array to a temporary array
		int temp[MedianFilterFactor]={0};
		for(int i = 0; i < MedianFilterFactor; i++) {
			temp[i] = ArrayHolderAux[i];
		}		
    	// Step 2: Sort the temporary array
		this->IntBubbleSort(temp,MedianFilterFactor);
    	// If odd, middle number
		return temp[MedianFilterFactor/2];*/

		// Efficient code
		// Step 0. Copy the array otherwise the original is modified!
		int ArrayHolderAuxTemp[MedianFilterFactor];
		for(int i = 0; i < MedianFilterFactor; i++) {
			ArrayHolderAuxTemp[i] = ArrayHolderAux[i];
		}
		// Step 1: Find the median element without fully sorting
	    int midIndex = MedianFilterFactor / 2;
	    std::nth_element(ArrayHolderAuxTemp, ArrayHolderAuxTemp + midIndex, ArrayHolderAuxTemp + MedianFilterFactor);

	    // Step 2: Return the median
	    return ArrayHolderAuxTemp[midIndex];
	}
}

// Function to implement Bubble Sort
int GPIO::IntBubbleSort(int* arr,int MedianFilterFactor) {
	int temp=0;
	for (int i = 0; i < MedianFilterFactor-1; i++) {
		for (int j = 0; j < MedianFilterFactor-i-1; j++) {
			if (arr[j] > arr[j+1]) {
                // Swap arr[j] and arr[j+1]
				temp = arr[j];
				arr[j] = arr[j+1];
				arr[j+1] = temp;
			}
		}
	}
    return 0; // All ok
}

long long int GPIO::LLIMeanFilterSubArray(long long int* ArrayHolderAux,int MeanFilterFactor){
	if (MeanFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
	// Step 1: Copy the array to a temporary array
		long long int temp=0.0;
		for(int i = 0; i < MeanFilterFactor; i++) {
			temp = temp + ArrayHolderAux[i];
		}
		
		temp=static_cast<long long int>(static_cast<long double>(temp)/static_cast<long double>(MeanFilterFactor));
		return temp;
	}
}

long double GPIO::LongDoubleMeanFilterSubArray(long double* ArrayHolderAux,int MeanFilterFactor){
	if (MeanFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
		long double temp=0.0;
		for(int i = 0; i < MeanFilterFactor; i++) {
			temp += ArrayHolderAux[i];
		}
		
		return temp/static_cast<long double>(MeanFilterFactor);
	}
}

long long int GPIO::LLIMedianFilterSubArray(long long int* ArrayHolderAux,int MedianFilterFactor){
	if (MedianFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
		/* // Non-efficient code
		// Step 1: Copy the array to a temporary array
		long double temp[MedianFilterFactor]={0.0};
		for(int i = 0; i < MedianFilterFactor; i++) {
			temp[i] = ArrayHolderAux[i];
		}
		
    	// Step 2: Sort the temporary array
		this->LongDoubleBubbleSort(temp,MedianFilterFactor);
    	// If odd, middle number
		return temp[MedianFilterFactor/2];*/

		// Efficient code
		// Step 0. Copy the array otherwise the original is modified!
		long long int ArrayHolderAuxTemp[MedianFilterFactor];
		for(int i = 0; i < MedianFilterFactor; i++) {
			ArrayHolderAuxTemp[i] = ArrayHolderAux[i];
		}
		// Step 1: Find the median element without fully sorting
	    int midIndex = MedianFilterFactor / 2;
	    std::nth_element(ArrayHolderAuxTemp, ArrayHolderAuxTemp + midIndex, ArrayHolderAuxTemp + MedianFilterFactor);

	    // Step 2: Return the median
	    return ArrayHolderAuxTemp[midIndex];
	}
}

long double GPIO::LongDoubleMedianFilterSubArray(long double* ArrayHolderAux,int MedianFilterFactor){
	if (MedianFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
		/* // Non-efficient code
		// Step 1: Copy the array to a temporary array
		long double temp[MedianFilterFactor]={0.0};
		for(int i = 0; i < MedianFilterFactor; i++) {
			temp[i] = ArrayHolderAux[i];
		}
		
    	// Step 2: Sort the temporary array
		this->LongDoubleBubbleSort(temp,MedianFilterFactor);
    	// If odd, middle number
		return temp[MedianFilterFactor/2];*/

		// Efficient code
		// Step 0. Copy the array otherwise the original is modified!
		long double ArrayHolderAuxTemp[MedianFilterFactor];
		for(int i = 0; i < MedianFilterFactor; i++) {
			ArrayHolderAuxTemp[i] = ArrayHolderAux[i];
		}
		// Step 1: Find the median element without fully sorting
	    int midIndex = MedianFilterFactor / 2;
	    std::nth_element(ArrayHolderAuxTemp, ArrayHolderAuxTemp + midIndex, ArrayHolderAuxTemp + MedianFilterFactor);

	    // Step 2: Return the median
	    return ArrayHolderAuxTemp[midIndex];
	}
}

double GPIO::DoubleMedianFilterSubArray(double* ArrayHolderAux,int MedianFilterFactor){
	if (MedianFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
		/* // Non-efficient code
		// Step 1: Copy the array to a temporary array
		double temp[MedianFilterFactor]={0.0};
		for(int i = 0; i < MedianFilterFactor; i++) {
			temp[i] = ArrayHolderAux[i];
		}		
	    // Step 2: Sort the temporary array
		this->DoubleBubbleSort(temp,MedianFilterFactor);
	    // If odd, middle number
		return temp[MedianFilterFactor/2];*/

		// Efficient code
		// Step 0. Copy the array otherwise the original is modified!
		double ArrayHolderAuxTemp[MedianFilterFactor];
		for(int i = 0; i < MedianFilterFactor; i++) {
			ArrayHolderAuxTemp[i] = ArrayHolderAux[i];
		}
		// Step 1: Find the median element without fully sorting
	    int midIndex = MedianFilterFactor / 2;
	    std::nth_element(ArrayHolderAuxTemp, ArrayHolderAuxTemp + midIndex, ArrayHolderAuxTemp + MedianFilterFactor);

	    // Step 2: Return the median
	    return ArrayHolderAuxTemp[midIndex];
	}
}

// Function to implement Bubble Sort
int GPIO::LongDoubleBubbleSort(long double* arr,int MedianFilterFactor) {
	long double temp=0.0;
	for (int i = 0; i < MedianFilterFactor-1; i++) {
		for (int j = 0; j < MedianFilterFactor-i-1; j++) {
			if (arr[j] > arr[j+1]) {
                // Swap arr[j] and arr[j+1]
				temp = arr[j];
				arr[j] = arr[j+1];
				arr[j+1] = temp;
			}
		}
	}
    return 0; // All ok
}

// Function to implement Bubble Sort
int GPIO::DoubleBubbleSort(double* arr,int MedianFilterFactor) {
	double temp=0.0;
	for (int i = 0; i < MedianFilterFactor-1; i++) {
		for (int j = 0; j < MedianFilterFactor-i-1; j++) {
			if (arr[j] > arr[j+1]) {
                // Swap arr[j] and arr[j+1]
				temp = arr[j];
				arr[j] = arr[j+1];
				arr[j+1] = temp;
			}
		}
	}
    return 0; // All ok
}

double GPIO::DoubleMeanFilterSubArray(double* ArrayHolderAux,int MeanFilterFactor){
	if (MeanFilterFactor<=1){
		return ArrayHolderAux[0];
	}
	else{
	// Step 1: Copy the array to a temporary array
		double temp=0.0;
		for(int i = 0; i < MeanFilterFactor; i++) {
			temp = temp + ArrayHolderAux[i];
		}
		
		temp=temp/((double)(MeanFilterFactor));
		return temp;
	}
}

/*****************************************************************************
* Local Function Definitions                                                 *
*****************************************************************************/

int GPIO::LOCAL_DDMinit(){
    //void *DDR_regaddr1, *DDR_regaddr2, *DDR_regaddr3;    
    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);// Maps the PRU shared RAM memory is then accessed by an array.
    sharedMem_int = (unsigned int*) sharedMem;
    
    prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru0dataMem);// Maps the PRU0 DRAM memory to input pointer. Memory is then accessed by an array.
    pru0dataMem_int = (unsigned int*) pru0dataMem;
    
    prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, &pru1dataMem);// Maps the PRU1 DRAM memory to input pointer. Memory is then accessed by an array.
    pru1dataMem_int = (unsigned int*) pru1dataMem;
    /*
    // open the device 
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        perror("Failed to open /dev/mem: ");
        return -1;
    }	

    // map the DDR memory
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, DDR_BASEADDR);
    if (ddrMem == NULL) {
        perror("Failed to map the device: ");
        close(mem_fd);
        return -1;
    }*/
    /*
    mem_fd = open ("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        printf ("ERROR: could not open /dev/mem.\n\n");
        return -1;
    }
    pru_int = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU_ADDR);
    if (pru_int == MAP_FAILED) {
        printf ("ERROR: could not map memory.\n\n");
        return -1;
    }*/
    
    //pru0dataMem_int =     (unsigned int*)pru_int + PRU0_DATARAM/4 + DATARAMoffset/4;   // Points to 0x200 of PRU0 memory
    //pru1dataMem_int =     (unsigned int*)pru_int + PRU1_DATARAM/4 + DATARAMoffset/4;   // Points to 0x200 of PRU1 memory
    //sharedMem_int   = 	  (unsigned int*)pru_int + SHAREDRAM/4; // Points to start of shared memory
    /*
    prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru0dataMem);// Maps the PRU0 DRAM memory to input pointer. Memory is then accessed by an array.
    pru0dataMem_int = (unsigned int*)pru0dataMem;// + DATARAMoffset/4;
    
    sharedMem_int = (unsigned int*)pru0dataMem + SHAREDRAM/4;
    
    prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, &pru1dataMem);// Maps the PRU1 DRAM memory to input pointer. Memory is then accessed by an array.
    pru1dataMem_int = (unsigned int*)pru1dataMem;// + DATARAMoffset/4;   
    */

    return 0;
}

int GPIO::KillcodePRUs(){
	if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/BBBhw/PRUkillSignal1.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/BBBhw/PRUassTrigSigScript.bin") == -1){
		if (prussdrv_exec_program(PRU_Signal_NUM, "./BBBhw/PRUkillSignal1.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./BBBhw/PRUassTrigSigScript.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUkillSignal1.bin");//perror("prussdrv_exec_program non successfull writing of PRUassTrigSigScript.bin");
		}
	}

	if (prussdrv_exec_program(PRU_Operation_NUM, "./CppScripts/BBBhw/PRUkillSignal0.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/BBBhw/PRUassTrigSigScript.bin") == -1){
		if (prussdrv_exec_program(PRU_Operation_NUM, "./BBBhw/PRUkillSignal0.bin") == -1){//if (prussdrv_exec_program(PRU_Signal_NUM, "./BBBhw/PRUassTrigSigScript.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUkillSignal0.bin");//perror("prussdrv_exec_program non successfull writing of PRUassTrigSigScript.bin");
		}
	}
	sleep(2); // Give time to load to the PRU memory
return 0;
}

int GPIO::DisablePRUs(){
// Disable PRU and close memory mappings
	prussdrv_pru_disable(PRU_Signal_NUM);
	prussdrv_pru_disable(PRU_Operation_NUM);

	return 0;
}

GPIO::~GPIO() { // Destructor
//	this->unexportGPIO();
	this->threadRefSynch.join();
	this->KillcodePRUs();
	this->DisablePRUs();
	//fclose(outfile); 
	prussdrv_exit();
	close(tfd);// close the time descriptor
	//munmap(ddrMem, 0x0FFFFFFF);
	//close(mem_fd); // Device
}

} /* namespace exploringBB */


using namespace exploringBB;

int main(int argc, char const * argv[]){
 // argv and argc are how command line arguments are passed to main() in C and C++.

 // argc will be the number of strings pointed to by argv. This will (in practice) be 1 plus the number of arguments, as virtually all implementations will prepend the name of the program to the array.

 // The variables are named argc (argument count) and argv (argument vector) by convention, but they can be given any valid identifier: int main(int num_args, char** arg_strings) is equally valid.

 // They can also be omitted entirely, yielding int main(), if you do not intend to process command line arguments.
 
 //printf( "argc:     %d\n", argc );
 //printf( "argv[0]:  %s\n", argv[0] );
 
 //if ( argc == 1 ) {
 // printf( "No arguments were passed.\n" );
 //}
 //else{
 // printf( "Arguments:\n" );
 // for (int i = 1; i < argc; ++i ) {
 //  printf( "  %d. %s\n", i, argv[i] );
 // }
 //}
 
 cout << "GPIOspadSYScont started..." << endl;
 
 GPIO GPIOagent; // Initiate the instance
 
 GPIOagent.m_start(); // Initiate in start state.
 
 /// Errors handling
 signal(SIGINT, SignalINTHandler);// Interruption signal
 signal(SIGPIPE, SignalPIPEHandler);// Error trying to write/read to a socket
 //signal(SIGSEGV, SignalSegmentationFaultHandler);// Segmentation fault
 
 bool isValidWhileLoop=true;
 if (GPIOagent.getState()==GPIO::APPLICATION_EXIT){isValidWhileLoop = false;}
 else{isValidWhileLoop = true;}
 
 //CKPDagent.GenerateSynchClockPRU();// Launch the generation of the clock
 cout << "Starting to actively adjust clock output..." << endl;
 
 while(isValidWhileLoop){ 
 	//CKPDagent.acquire();
   //try{
 	//try {
    	// Code that might throw an exception 
 	// Check if there are need messages or actions to be done by the node
 	
       switch(GPIOagent.getState()) {
           case GPIO::APPLICATION_RUNNING: {               
               // Do Some Work
               //GPIOagent.HandleInterruptSynchPRU();
               break;
           }
           case GPIO::APPLICATION_PAUSED: {
               // Maybe do some checks if necessary 
               break;
           }
           case GPIO::APPLICATION_EXIT: {                  
               isValidWhileLoop=false;//break;
           }
           default: {
               // ErrorHandling Throw An Exception Etc.
           }

        } // switch
        
	if (signalReceivedFlag.load()){GPIOagent.~GPIO();}// Destroy the instance
        // Main barrier is in HandleInterruptSynchPRU function. No need for this CKPDagent.RelativeNanoSleepWait((unsigned int)(WaitTimeAfterMainWhileLoop));
        
    //}
    //catch (const std::exception& e) {
    //	// Handle the exception
    //	cout << "Exception: " << e.what() << endl;
    //	}
  //} // upper try
  //catch (...) { // Catches any exception
  //cout << "Exception caught" << endl;
  //  }
	//CKPDagent.release();
	//CKPDagent.RelativeNanoSleepWait((unsigned int)(WaitTimeAfterMainWhileLoop));// Used in busy-wait
    } // while
  cout << "Exiting GPIOspadSYScont" << endl;
  
  
 return 0; // Everything Ok
}
