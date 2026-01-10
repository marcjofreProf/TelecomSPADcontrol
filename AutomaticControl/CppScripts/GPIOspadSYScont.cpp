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
#include<cstring> // For memset
#include<cstdint> // for SPI data
#include<iomanip>
#include<cstdio>
#include<fcntl.h>
#include<unistd.h>
#include<sys/epoll.h>
#include<thread>
#include<pthread.h>
#include<unistd.h>
#include<algorithm> // For std::nth_element
#include<signal.h>
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
//int exploringBB::GPIO::mem_fd = -1;// Define and initialize 

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
	if (prussdrv_exec_program(PRU_Operation_NUM, "./CppScripts/PRUsignalReads.bin") == -1){
		if (prussdrv_exec_program(PRU_Operation_NUM, "./PRUsignalReads.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUsignalReads.bin");
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
	if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/PRUsignalWrites.bin") == -1){
		if (prussdrv_exec_program(PRU_Signal_NUM, "./PRUsignalWrites.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUsignalWrites.bin");
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
	// Initiate SPI communications
	const char* spi_device = "/dev/spidev1.0";  // SPI1 bus, CS0
	// Open SPI device
	spi_fd = open(spi_device, O_RDWR);
	//printf("CONFIG fd = %d\n", spi_fd);
	if (spi_fd < 0) {
	    cout << "Failed to open SPI device" << endl;
	}

	// 1. Set SPI mode (0, 1, 2, or 3)
	uint8_t spi_mode = 0;  // CPOL=0, CPHA=0
	if (ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
	    cout << "Failed to set SPI mode" << endl;
	}

	// 2. Set bits per word (usually 8)
	uint8_t bits_per_word = 8;
	if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
	    cout << "Failed to set bits per word" << endl;
	}

	// 3. Set maximum speed (in Hz)
	uint32_t spi_speed = 5000;  // 5 kHz
	if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
	    cout << "Failed to set SPI speed" << endl;
	}

	// Optional: Read back settings to verify
	ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode);
	ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word);
	ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);

	std::cout << "SPI Configuration:" << std::endl;
	std::cout << "  Mode: " << (int)spi_mode << std::endl;
	std::cout << "  Bits per word: " << (int)bits_per_word << std::endl;
	std::cout << "  Speed: " << spi_speed << " Hz" << std::endl;

	spiTransferByte(spi_fd, 0xFF); // Set initial value

	// Set initial Time Wall
	TimePointClockCurrentSynchPRU0future=ClockChrono::now()+std::chrono::nanoseconds(WaitTimeAfterMainWhileLoop);
	
	auto duration_since_epochFutureTimePoint=TimePointClockCurrentSynchPRU0future.time_since_epoch();
	// Convert duration to desired time
	long long int TimePointClockCurrentFinal_time_as_count = static_cast<long long int>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epochFutureTimePoint).count());
	//cout << "TimePointClockCurrentFinal_time_as_count: " << TimePointClockCurrentFinal_time_as_count << endl;

	requestTimeWait.tv_sec=(int)(TimePointClockCurrentFinal_time_as_count/((long)1000000000));
	requestTimeWait.tv_nsec=(long)(TimePointClockCurrentFinal_time_as_count%(long)1000000000);
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
// SPI communicatoin operations
uint8_t GPIO::spiTransferByte(int spi_fdAux, uint8_t tx){
    uint8_t rx = 0;
    spi_ioc_transfer tr{};
    //printf("XFER fd = %d\n", spi_fdAux);
    tr.tx_buf = reinterpret_cast<uint64_t>(&tx);
    tr.rx_buf = reinterpret_cast<uint64_t>(&rx);
    tr.len = 1;
    tr.bits_per_word = 8;

    int ret = ioctl(spi_fdAux, SPI_IOC_MESSAGE(1), &tr);
    if (ret != 1) {
        perror("SPI_IOC_MESSAGE");
    }

    return rx;
}


// Function to ramp SPI voltage to desired value beggining from the current SPI value
int GPIO::SPIrampVoltage(int spi_fdAux, float desired_voltage, float max_rate, bool verbose) {
    
    if (verbose) cout << "\033[2J\033[1;1H";
    
    // Validate input
    if (desired_voltage < MIN_V || desired_voltage > MAX_V) return -1;
    if (max_rate <= 0) return -2;
    
    // Check if already at target (within 0.25V tolerance)
    if (fabs(currentSPIvalue - desired_voltage) < 0.25) {
        currentSPIvalue = desired_voltage;
        if (verbose) cout << "At target: " << fixed << setprecision(2) << desired_voltage << "V" << endl;
        return 0;
    }
    
    // Convert voltages to SPI values
    int current_spi = 255 - (int)((currentSPIvalue - MIN_V) / RATIO);
    int target_spi = 255 - (int)((desired_voltage - MIN_V) / RATIO);
    
    // Clamp
    if (current_spi < 0) current_spi = 0;
    if (current_spi > 255) current_spi = 255;
    if (target_spi < 0) target_spi = 0;
    if (target_spi > 255) target_spi = 255;
    
    // Python algorithm calculations
    float step_range = (90.0 - 40.0) / 255.0;
    float decimation = 10.0;
    
    int spi_range = abs(target_spi - current_spi);
    int steps = (int)round(spi_range / step_range / decimation);
    if (steps < 1) steps = 1;
    
    float step_time = (step_range / max_rate) * decimation * 1000000;
    
    if (verbose) {
        cout << "Ramping: " << fixed << setprecision(2) << currentSPIvalue << "V -> " << fixed << setprecision(2) << desired_voltage << "V" << endl;
        cout << "Steps: " << steps << " | Time: " << (step_time * steps / 1000000.0) << "s" << endl;
        cout << "[";
    }
    
    // Perform ramp
    for (int i = 0; i <= steps; i++) {
        //if (signalReceivedFlag.load()) return -3; Do not check on this - finish the ramp safely
        
        float t = (float)i / steps;
        int spi_val = current_spi + (int)(t * (target_spi - current_spi));
        if (i == steps) spi_val = target_spi;
        
        if (spi_val < 0) spi_val = 0;
        if (spi_val > 255) spi_val = 255;
        
        spiTransferByte(spi_fdAux,(uint8_t)spi_val);
        //cout << "spi_val: 0x" << hex << spi_val << dec << endl;
        
        float voltage = MIN_V + (RATIO * (255 - spi_val));
        currentSPIvalue = voltage;
        
        if (verbose) {
            int percent = (i * 100) / steps;
            cout << "\r[";
            for (int j = 0; j < 40; j++) {
                if (j < percent * 40 / 100) cout << "=";
                else if (j == percent * 40 / 100) cout << ">";
                else cout << " ";
            }
            cout << "] " << percent << "% (" << fixed << setprecision(2) << voltage << "V)";
            cout.flush();
        }
        
        if (i < steps) usleep((useconds_t)step_time);
    }
    
    if (verbose) {
        cout << "\r\033[K";
        cout << "\r[========================================] 100% (" << fixed << setprecision(2) << desired_voltage << "V)" << endl;
    }
    
    currentSPIvalue = desired_voltage;
    return 0;
}
//////////////////////////////////////////////
int GPIO::NonBusyTimeWall(){
	clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&requestTimeWait,NULL); // Non-busy wait
	TimePointClockCurrentSynchPRU0future=TimePointClockCurrentSynchPRU0future+std::chrono::nanoseconds(WaitTimeAfterMainWhileLoop);
	
	auto duration_since_epochFutureTimePoint=TimePointClockCurrentSynchPRU0future.time_since_epoch();
	// Convert duration to desired time
	long long int TimePointClockCurrentFinal_time_as_count = static_cast<long long int>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epochFutureTimePoint).count());
	//cout << "TimePointClockCurrentFinal_time_as_count: " << TimePointClockCurrentFinal_time_as_count << endl;

	requestTimeWait.tv_sec=(int)(TimePointClockCurrentFinal_time_as_count/((long)1000000000));
	requestTimeWait.tv_nsec=(long)(TimePointClockCurrentFinal_time_as_count%(long)1000000000);

	return 0;
}

int GPIO::HandleInterruptPRUs(){ // Uses output pins to clock subsystems physically generating qubits or entangled qubits

//ReadTimeCounts();
// TODO: Do calculations with the counts retrieved
// Prepare adjustments and communicate to the signal PRU thorugh internal interrupts
//SendControlSignals(); // Already launched at the beggining
// Apply DC bias adjustments
//SPIrampVoltage(int spi_fd, desired_voltage_current, 2.0, false);

return 0;// All ok
}

int GPIO::ReadTimeCounts(){// Read the SPADs associated counters
	pru0dataMem_int[0]=static_cast<unsigned int>(1); // set command
	prussdrv_pru_send_event(21);
	int retInterruptsPRU0=prussdrv_pru_wait_event_timeout(PRU_EVTOUT_0,WaitTimeInterruptPRU0);

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

	this->DDRdumpdata(); // Pre-process tags. Needs to access memory of PRU, so better within the controlled acquired environment

	return 0;// all ok
}

int GPIO::SendControlSignals(){	
	pru1dataMem_int[0]=static_cast<unsigned int>(1); // set command. Generate signals.
	prussdrv_pru_send_event(22);//Send host arm to PRU1 interrupt
	// Here there should be the instruction command to tell PRU1 to start generating signals
	// We have to define a command, compatible with the memory space of PRU0 to tell PRU1 to initiate signals
	/*
	//  PRU long execution making sure that notification interrupts do not overlap
	int retInterruptsPRU1=prussdrv_pru_wait_event_timeout(PRU_EVTOUT_1,WaitTimeInterruptPRU1);

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
	*/
return 0;// all ok	
}

int GPIO::DDRdumpdata(){
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
    
    return 0;
}

int GPIO::RelativeNanoSleepWait(unsigned int TimeNanoSecondsSleep){
struct timespec ts;
ts.tv_sec=(int)(TimeNanoSecondsSleep/((long)1000000000));
ts.tv_nsec=(long)(TimeNanoSecondsSleep%(long)1000000000);
clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL); //

return 0; // All ok
}

int GPIO::KillcodePRUs(){
	if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/PRUkillSignal1.bin") == -1){
		if (prussdrv_exec_program(PRU_Signal_NUM, "./PRUkillSignal1.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUkillSignal1.bin");
		}
	}

	if (prussdrv_exec_program(PRU_Operation_NUM, "./CppScripts/PRUkillSignal0.bin") == -1){
		if (prussdrv_exec_program(PRU_Operation_NUM, "./PRUkillSignal0.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUkillSignal0.bin");
		}
	}	
return 0;
}

int GPIO::DisablePRUs(){
// Disable PRU and close memory mappings
	prussdrv_pru_disable(PRU_Signal_NUM);
	prussdrv_pru_disable(PRU_Operation_NUM);

	return 0;
}

GPIO::~GPIO() { // Destructor
	this->KillcodePRUs();
	cout << "Exiting GPIOspadSYScont..." << endl;
	// Finish with lowering the bias voltage  
	spiTransferByte(spi_fd, 0xFF); // Set final value	
	sleep(0.5); // Give time to load to the PRU memory and send the values to spi
	close(spi_fd); // Close SPI file descriptor
	//this->threadRefSynch.join();	
	this->DisablePRUs();
	prussdrv_exit();
	cout << "Exit GPIOspadSYScont done!" << endl;
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
 
 float initialDesiredDCvoltage=55.0;
 if ( argc == 1 ) {
 	cout << "No arguments were passed!" << endl;
 }
 else{
	 //cout << "Arguments" << endl;
	 //for (int i = 1; i < argc; ++i ) {
	 //	printf( "  %d. %s\n", i, argv[i] );
	 //}
 	initialDesiredDCvoltage=stof(argv[1]);
 }
 
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
 // First initial volage bias up
 GPIOagent.SPIrampVoltage(GPIOagent.spi_fd, initialDesiredDCvoltage, 2.0, true);
 GPIOagent.SendControlSignals();
 
 while(isValidWhileLoop && !signalReceivedFlag.load()){ 
 	//CKPDagent.acquire();
   //try{
 	//try {
    // Code that might throw an exception 
 	// Check if there are need messages or actions to be done by the node
 	
       switch(GPIOagent.getState()) {
           case GPIO::APPLICATION_RUNNING: {               
               // Do Some Work
               GPIOagent.HandleInterruptPRUs();
               break;
           }
           case GPIO::APPLICATION_PAUSED: { // When no corrections are aimed (maybe because they are true signal detections periods)
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
        
	//if (signalReceivedFlag.load()){GPIOagent.~GPIO();}// Destroy the instance// done somewhere else
    // Time wall
    //GPIOagent.RelativeNanoSleepWait((unsigned int)(WaitTimeAfterMainWhileLoop)); // Like this, it will depend on how long in time the previous functions have lasted
    GPIOagent.NonBusyTimeWall();// Used with non-busy wait
        
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
	//CKPDagent.RelativeNanoSleepWait((unsigned int)(WaitTimeAfterMainWhileLoop));
    } // while
  
 return 0; // Everything Ok
}
