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
#include <termios.h> // Keyboard detection in foreground
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
	CalpHolder=(unsigned int*)&sharedMem_int[OFFSET_SHAREDRAM];
	
	// Launch the PRU0 and PRU1 codes but put them in idle mode, waiting for command
	// Reading counters
	// Execute program
	// Load and execute the PRU program on the PRU0	
	pru0dataMem_int[0]=static_cast<unsigned int>(0); // set no command
	pru0dataMem_int[1]=static_cast<unsigned int>(PRUmeasInterval/PRUclockStepPeriodNanoseconds); // Measurement time interval in clock cycles. Very important to be a power of 2.
	cout << "pru0dataMem_int[1]: " << pru0dataMem_int[1] << endl;

	if (prussdrv_exec_program(PRU_Operation_NUM, "./CppScripts/PRUsignalReads.bin") == -1){
		if (prussdrv_exec_program(PRU_Operation_NUM, "./PRUsignalReads.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUsignalReads.bin");
		}
	}
	////prussdrv_pru_enable(PRU_Operation_NUM);
	
	// Generate signals	
	pru1dataMem_int[0]=static_cast<unsigned int>(0); // set no command
	pru1dataMem_int[1]=static_cast<unsigned int>(pru1_cycles_period); // Initial number of clocks per cycle (it has to be power of 2)
	pru1dataMem_int[2]=static_cast<unsigned int>(pru1_delay_first_off); // Initial number of relative clocks/2 - 1 of the first off. At least 1.
	pru1dataMem_int[3]=static_cast<unsigned int>(pru1_delay_second_off); // Initial number of relative clocks/2 - 1 of the second off. At least 1.
	pru1dataMem_int[4]=static_cast<unsigned int>(pru1_delay_third_off); // Initial number of relative clocks/2 - 1 of the third off. At least 1.
	pru1dataMem_int[5]=static_cast<unsigned int>(pru1_delay_fourth_off); // Initial number of relative clocks/2 - 1 of the fourth off. At least 1.
	pru1dataMem_int[6]=static_cast<unsigned int>(pru1_mask_first_off); // Initial mask of the first off
	pru1dataMem_int[7]=static_cast<unsigned int>(pru1_mask_second_off); // Initial mask of the second also off
	pru1dataMem_int[8]=static_cast<unsigned int>(pru1_mask_third_off); // Initial mask of the third also off
	pru1dataMem_int[9]=static_cast<unsigned int>(pru1_mask_fourth_off); // Initial mask of the fourth also off
	// Load and execute the PRU program on the PRU1
	if (prussdrv_exec_program(PRU_Signal_NUM, "./CppScripts/PRUsignalWrites.bin") == -1){
		if (prussdrv_exec_program(PRU_Signal_NUM, "./PRUsignalWrites.bin") == -1){
			perror("prussdrv_exec_program non successfull writing of PRUsignalWrites.bin");
		}
	}

	////prussdrv_pru_enable(PRU_Signal_NUM);
	sleep(1); // Give some time to load programs in PRUs and the synch protocols to initiate and lock after prioritazion and adjtimex. Very important, otherwise bad values might be retrieved
	////////////////////////////////////////////////////////
	// Some variables initializations
	for (int i=0;i<NumDetChannels;i++){
		duty_cycles[i]=AVG_DUTY;  // Current duty cycles, will be updated
		duty_integrals[i]=0.0;  // Need to maintain for each channel
		duty_prev_errors[i]=0.0;  // Need to maintain for each channel
	}
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
///////////////////////////////
/// Errors handling
std::atomic<bool> signalReceivedFlag{false};
static void SignalINTHandler(int s) {
signalReceivedFlag.store(true);
cout << "Caught SIGINT" << endl;
}

static void SignalTERMHandler(int s) {
signalReceivedFlag.store(true);
cout << "Caught SIGTERM" << endl;
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
    
    //if (verbose) cout << "\033[2J\033[1;1H";
    
    // Validate input
    if (desired_voltage < MIN_V || desired_voltage > MAX_V) return -1;
    if (max_rate <= 0) return -2;
    
    // Check if already at target (within 0.2V tolerance)
    if (fabs(currentVoltageValue - desired_voltage) < MIN_SPI_V_STEP && (abs(voltage_error)<=voltage_error_thresholdPercent)) {
        //currentVoltageValue = desired_voltage; // Dangerous because it could actually not be upated
        if (verbose) cout << "At target: " << fixed << setprecision(2) << currentVoltageValue << "V" << endl;
        this->m_pause();
	    //cout << "System paused. Press any key to resume..." << endl;
        return 0;
    }
    
    // Convert voltages to SPI values
    int current_spi = 255 - (int)((currentVoltageValue - MIN_V) / RATIO);
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
        cout << "Ramping: " << fixed << setprecision(2) << currentVoltageValue << "V -> " << fixed << setprecision(2) << desired_voltage << "V" << endl;
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
        currentVoltageValue = voltage;
        
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
        //cout << "\r\033[K";
        cout << "\r[========================================] 100% (" << fixed << setprecision(2) << desired_voltage << "V)" << endl;
    }
    
    currentVoltageValue = desired_voltage;
    return 0;
}
//////////////////////////////////////////////
int GPIO::NonBusyTimeWall(){
	clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&requestTimeWait,NULL); // Non-busy wait
	// Check if we need to resync (time may have already passed)
	auto now = ClockChrono::now();
	if (now >= TimePointClockCurrentSynchPRU0future){
		SetNewFutureTimeWall(); // Recompute synchronization
	}
	else{
		TimePointClockCurrentSynchPRU0future=TimePointClockCurrentSynchPRU0future+std::chrono::nanoseconds(WaitTimeAfterMainWhileLoop);
	}
	
	auto duration_since_epochFutureTimePoint=TimePointClockCurrentSynchPRU0future.time_since_epoch();
	// Convert duration to desired time
	long long int TimePointClockCurrentFinal_time_as_count = static_cast<long long int>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epochFutureTimePoint).count());
	//cout << "TimePointClockCurrentFinal_time_as_count: " << TimePointClockCurrentFinal_time_as_count << endl;

	requestTimeWait.tv_sec=(int)(TimePointClockCurrentFinal_time_as_count/((long)1000000000));
	requestTimeWait.tv_nsec=(long)(TimePointClockCurrentFinal_time_as_count%(long)1000000000);

	return 0;
}

// PID function accounting for non-linear response of the SPADs (at some point the more voltage the less counts, because we have passed the inflection point)
int GPIO::calculateSPADControl(){
    current_desired_voltage=currentVoltageValue; // Update value
    // Calculate counts per second for each channel
    double total_cps = 0.0;
    double numChactive=0.0;
    for(int i = 0; i < NumDetChannels; i++) {
        if (DetCounterCh[i]>0){
            total_cps += ((double)DetCounterCh[i]) / ((double)DT);
            numChactive++;
        }
    }
    
    // Calculate average and voltage error
    double avg_cps=0; // Initialization    
    if (numChactive>0){
        avg_cps = total_cps / numChactive;
        voltage_error = (TARGET_CPS - avg_cps) / (TARGET_CPS);
        
        // Print the 4 individual conditions
		cout << "Condition 1: " << (abs(voltage_error) > voltage_error_thresholdPercent ? "true" : "false") 
    	 << " (|voltage_error|=" << abs(voltage_error) << " > threshold=" << voltage_error_thresholdPercent << ")" << endl;

		cout << "Condition 2: " << (abs(current_desired_voltage - last_voltage) < MIN_SPI_V_STEP ? "true" : "false") 
    	 << " (|diff|=" << abs(current_desired_voltage - last_voltage) << " < MIN_SPI_V_STEP=" << MIN_SPI_V_STEP << ")" << endl;

		cout << "Condition 3: " << ((current_desired_voltage - last_voltage) >= 0.4 ? "true" : "false") 
    	 << " (diff=" << (current_desired_voltage - last_voltage) << " >= 0.4)" << endl;

		cout << "Condition 4: " << (avg_cps <= last_avg_cps * 0.8 ? "true" : "false") 
    	 << " (avg_cps=" << avg_cps << " <= last_avg_cps*0.8=" << last_avg_cps * 0.8 << ")" << endl;

        // NEW: Check if we're past the inflection point (requires 3 consecutive detections)
        if (abs(voltage_error)>voltage_error_thresholdPercent && (abs(current_desired_voltage - last_voltage) < MIN_SPI_V_STEP || (current_desired_voltage - last_voltage) >= 0.4) && avg_cps <= last_avg_cps * 0.8) {
        	//if (total_cps>0.0){ // Update values if different than 0
            //	inflection_counter++;
            //}
            //else{
            //	inflection_counter=0; // reset counter
            //}

            inflection_counter++;

            if (inflection_counter >= 8) { // Number of checks to consider that it has surpassed the inflection point
                past_inflection_point = true;
                cout << "Detected passing voltage inflection point. Implementing voltage drop correction!"<< endl;
            }
        }
        else {
            // Reset counter if condition not met
            past_inflection_point = false;
            inflection_counter = 0;
        }        
    }
    else{
        voltage_error = 1.0; // 100% error - we want MORE voltage to get counts
        avg_cps = 0; // Average value
        voltage_prev_error = 0.0; // It has to be reset to zero if no counts for the algorithm to advance
        voltage_integral = 0.0; // It has to be reset to zero if no counts for the algorithm to advance
        past_inflection_point = false; // Reset flag when no counts
    }

    // Store for next iteration
	last_avg_cps = avg_cps;
	last_voltage = current_desired_voltage;
    
    // Voltage PID calculation
    if (abs(voltage_error)>voltage_error_thresholdPercent){// Only change PID value if the error is larger than 10% or if no average counts
        double P_voltage = Kp_voltage * voltage_error;
        
        voltage_integral += voltage_error * DT;
        if(voltage_integral > voltage_integral_limit) voltage_integral = voltage_integral_limit;
        if(voltage_integral < -voltage_integral_limit) voltage_integral = -voltage_integral_limit;
        double I_voltage = Ki_voltage * voltage_integral;
        
        double D_voltage = Kd_voltage * (voltage_error - voltage_prev_error) / DT;
        voltage_prev_error = voltage_error;
        
        double voltage_adj = P_voltage + I_voltage + D_voltage;       
        
        // Limit voltage step
        if(voltage_adj > MAX_V_STEP) voltage_adj = MAX_V_STEP;
        if(voltage_adj < -MAX_V_STEP) voltage_adj = -MAX_V_STEP;

        // NEW: Force voltage reduction if past inflection point
        if (past_inflection_point) {
            current_desired_voltage = initialDesiredDCvoltage; // Force maximum reduction
            // Reset integral to prevent windup
            voltage_integral = 0;
        }
        else{
        	// Update voltage
        	current_desired_voltage += voltage_adj;
        }
        
        // Clamp voltage
        if(current_desired_voltage < MIN_VOLTAGE) current_desired_voltage = MIN_VOLTAGE;
        if(current_desired_voltage > MAX_VOLTAGE) current_desired_voltage = MAX_VOLTAGE;
    }
    
    // ... rest of your duty cycle code remains exactly the same ...
    
    return 0;
}
int GPIO::updatePRU1values(){
	// Calculate when each channel turns OFF (in cycles from start)
    unsigned int off_time[NumDetChannels];
    for (int i = 0; i < NumDetChannels; i++) {
        off_time[i] = static_cast<unsigned int>(duty_cycles[i] * (double)pru1_cycles_period);
    }
    
    // Sort channels by turn-off time (earliest first)
    // ADD TIE-BREAKER: when times are equal, sort by channel number
    int order[NumDetChannels] = {0, 1, 2, 3};
    for (int i = 0; i < NumDetChannels; i++) {
        for (int j = i + 1; j < NumDetChannels; j++) {
            // FIX: Compare both time AND channel number for tie-breaking
            if (off_time[order[i]] > off_time[order[j]] || 
                (off_time[order[i]] == off_time[order[j]] && order[i] > order[j])) {
                int temp = order[i];
                order[i] = order[j];
                order[j] = temp;
            }
        }
    }
    
    unsigned int channels_off = 0;  // Bits 0-3: 1 = channel turned off
    
    // First turn-off
    channels_off |= (1u << order[0]);  // Mark channel as turned off
    pru1_mask_first_off = 0x0F & ~channels_off;  // Clear all turned-off channels
    pru1_delay_first_off = off_time[order[0]];
    
    // Second turn-off
    if (off_time[order[1]] > pru1_delay_first_off) {
        pru1_delay_second_off = off_time[order[1]] - pru1_delay_first_off;
        channels_off |= (1u << order[1]);  // Mark this channel as turned off
        pru1_mask_second_off = 0x0F & ~channels_off;
    } else {
        pru1_delay_second_off = 1;  // 3/2-1 = 1
        channels_off |= (1u << order[1]);  // Mark as turned off
        pru1_mask_first_off = 0x0F & ~channels_off;  // Update first mask
        pru1_mask_second_off = pru1_mask_first_off;  // Same mask (both turned off at same time)
    }
    
    // Third turn-off
    unsigned int time_to_third = pru1_delay_first_off + pru1_delay_second_off;
    if (off_time[order[2]] > time_to_third) {
        pru1_delay_third_off = off_time[order[2]] - time_to_third;
        channels_off |= (1u << order[2]);  // Mark as turned off
        pru1_mask_third_off = 0x0F & ~channels_off;
    } else {
        pru1_delay_third_off = 1;
        channels_off |= (1u << order[2]);  // Mark as turned off
        
        // Update the mask where this channel turns off
        if (off_time[order[2]] <= pru1_delay_first_off) {
            // Turns off with first group
            pru1_mask_first_off = 0x0F & ~channels_off;
            pru1_mask_third_off = pru1_mask_first_off;
            pru1_mask_second_off = pru1_mask_first_off;  // Also update second if it was same time
        } else {
            // Turns off with second group
            pru1_mask_second_off = 0x0F & ~channels_off;
            pru1_mask_third_off = pru1_mask_second_off;
        }
    }
    
    // Fourth turn-off
    unsigned int time_to_fourth = time_to_third + pru1_delay_third_off;
    if (off_time[order[3]] > time_to_fourth) {
        pru1_delay_fourth_off = off_time[order[3]] - time_to_fourth;
        channels_off |= (1u << order[3]);  // Mark as turned off
        pru1_mask_fourth_off = 0x0F & ~channels_off;
    } else {
        pru1_delay_fourth_off = 1;
        channels_off |= (1u << order[3]);  // Mark as turned off
        
        // Update the appropriate mask
        if (off_time[order[3]] <= pru1_delay_first_off) {
            pru1_mask_first_off = 0x0F & ~channels_off;
            pru1_mask_fourth_off = pru1_mask_first_off;
            pru1_mask_second_off = pru1_mask_first_off;
            pru1_mask_third_off = pru1_mask_first_off;
        } else if (off_time[order[3]] <= time_to_third) {
            pru1_mask_second_off = 0x0F & ~channels_off;
            pru1_mask_fourth_off = pru1_mask_second_off;
            pru1_mask_third_off = pru1_mask_second_off;
        } else {
            pru1_mask_third_off = 0x0F & ~channels_off;
            pru1_mask_fourth_off = pru1_mask_third_off;
        }
    }

    // Update relative delays to PRU implementation
    if (pru1_delay_first_off > 1) pru1_delay_first_off = pru1_delay_first_off / 2 - 1;
    if (pru1_delay_second_off > 1) pru1_delay_second_off = pru1_delay_second_off / 2 - 1;
    if (pru1_delay_third_off > 1) pru1_delay_third_off = pru1_delay_third_off / 2 - 1;
    if (pru1_delay_fourth_off > 1) pru1_delay_fourth_off = pru1_delay_fourth_off / 2 - 1;
    
    // Ensure minimum delay of 1 cycle
    pru1_delay_first_off = (pru1_delay_first_off < 1) ? 1 : pru1_delay_first_off;
    pru1_delay_second_off = (pru1_delay_second_off < 1) ? 1 : pru1_delay_second_off;
    pru1_delay_third_off = (pru1_delay_third_off < 1) ? 1 : pru1_delay_third_off;
    pru1_delay_fourth_off = (pru1_delay_fourth_off < 1) ? 1 : pru1_delay_fourth_off;

    // Write values to PRU1 RAM memory
    pru1dataMem_int[1]=static_cast<unsigned int>(pru1_cycles_period); // Initial number of clocks per cycle (it has to be power of 2)
	pru1dataMem_int[2]=static_cast<unsigned int>(pru1_delay_first_off); // Initial number of relative clocks/2 - 1 of the first off. At least 1.
	pru1dataMem_int[3]=static_cast<unsigned int>(pru1_delay_second_off); // Initial number of relative clocks/2 - 1 of the second off. At least 1.
	pru1dataMem_int[4]=static_cast<unsigned int>(pru1_delay_third_off); // Initial number of relative clocks/2 - 1 of the third off. At least 1.
	pru1dataMem_int[5]=static_cast<unsigned int>(pru1_delay_fourth_off); // Initial number of relative clocks/2 - 1 of the fourth off. At least 1.
	pru1dataMem_int[6]=static_cast<unsigned int>(pru1_mask_first_off); // Initial mask of the first off
	pru1dataMem_int[7]=static_cast<unsigned int>(pru1_mask_second_off); // Initial mask of the second also off
	pru1dataMem_int[8]=static_cast<unsigned int>(pru1_mask_third_off); // Initial mask of the third also off
	pru1dataMem_int[9]=static_cast<unsigned int>(pru1_mask_fourth_off); // Initial mask of the fourth also off
	
	// Send host interupt to PRU1 - no command needed
	prussdrv_pru_send_event(22);//Send host arm to PRU1 interrupt

	return 0;
}

int GPIO::OperDataDebShow(){ // Show operationaldata
	// Debugging
	std::cout << std::dec; // Force decimal format
	cout << "DDRdumpdata DetCounterCh[0]: " << DetCounterCh[0] << endl;
	cout << "DDRdumpdata DetCounterCh[1]: " << DetCounterCh[1] << endl;
	cout << "DDRdumpdata DetCounterCh[2]: " << DetCounterCh[2] << endl;
	cout << "DDRdumpdata DetCounterCh[3]: " << DetCounterCh[3] << endl;

	cout << "current_desired_voltage: " << current_desired_voltage << "V" << endl;
	cout << "duty_cycles[0]: " << duty_cycles[0] << endl;
	cout << "duty_cycles[1]: " << duty_cycles[1] << endl;
	cout << "duty_cycles[2]: " << duty_cycles[2] << endl;
	cout << "duty_cycles[3]: " << duty_cycles[3] << endl;

	std::cout << "pru1_delay_first_off: " << pru1_delay_first_off << std::endl;
    std::cout << "pru1_delay_second_off: " << pru1_delay_second_off << std::endl;
    std::cout << "pru1_delay_third_off: " << pru1_delay_third_off << std::endl;
    std::cout << "pru1_delay_fourth_off: " << pru1_delay_fourth_off << std::endl;
    std::cout << "pru1_mask_first_off: 0x" << std::hex << pru1_mask_first_off << std::endl;
    std::cout << "pru1_mask_second_off: 0x" << std::hex << pru1_mask_second_off << std::endl;
    std::cout << "pru1_mask_third_off: 0x" << std::hex << pru1_mask_third_off << std::endl;
    std::cout << "pru1_mask_fourth_off: 0x" << std::hex << pru1_mask_fourth_off << std::endl;
    std::cout << std::dec; // Force decimal format again

	return 0;// All ok
}

int GPIO::HandleInterruptPRUsActive(){ // Uses output pins to clock subsystems physically generating qubits or entangled qubits
	//cout << "\033[2J\033[1;1H"; // Clear the terminal screen and move the cursor to the top row

	ReadTimeCounts(); // Read the counters of detections
	calculateSPADControl(); // Calculate the adjustmenst to do
	updatePRU1values();// Send to signal PRU duty cycle adjustments.
	SPIrampVoltage(spi_fd, current_desired_voltage, 2.0, true); // Apply DC bias adjustments // Verbose should be turn to false one debugging is complete

	OperDataDebShow();

	return 0;// All ok
}

int GPIO::HandleInterruptPRUsPaused(){ // Uses output pins to clock subsystems physically generating qubits or entangled qubits
	ReadTimeCounts(); // Read the counters of detections
	
	cout << "\033[2J\033[1;1H"; // Clear the terminal screen and move the cursor to the top row
	cout << "At target: " << fixed << setprecision(2) << currentVoltageValue << "V" << endl;
	OperDataDebShow();

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

	this->DDRdumpdata(); // Pre-process counter. Needs to access memory of PRU, so better within the controlled acquired environment

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
valp=CalpHolder; // Coincides with SHARED in PRUsignalReads.p

// When unsigned int
DetCounterCh[0]=static_cast<unsigned int>(*valp);
valp++;
DetCounterCh[1]=static_cast<unsigned int>(*valp);
valp++;
DetCounterCh[2]=static_cast<unsigned int>(*valp);
valp++;
DetCounterCh[3]=static_cast<unsigned int>(*valp);

// Debugging
//cout << "DDRdumpdata DetCounterCh[0]: " << DetCounterCh[0] << endl;
//cout << "DDRdumpdata DetCounterCh[1]: " << DetCounterCh[1] << endl;
//cout << "DDRdumpdata DetCounterCh[2]: " << DetCounterCh[2] << endl;
//cout << "DDRdumpdata DetCounterCh[3]: " << DetCounterCh[3] << endl;

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

int GPIO::SetNewFutureTimeWall(){
	TimePointClockCurrentSynchPRU0future=ClockChrono::now()+std::chrono::nanoseconds(WaitTimeAfterMainWhileLoop);
	return 0;
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

	if (!isatty(STDIN_FILENO)) cout << "STDIN is NOT a TTY" << endl;
	else cout << "STDIN is a TTY" << endl;

	// Save terminal settings
	termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	// NON-CANONICAL MODE + NO ECHO
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_cc[VMIN] = 0;  
	newt.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	char KeyboardC;

	GPIO GPIOagent; // Initiate the instance
	 
	 GPIOagent.m_start(); // Initiate in start state.
	 
	 /// Errors/actions handling
	 signal(SIGINT, SignalINTHandler);// Interruption signal
	 signal(SIGTERM, SignalTERMHandler); // kill, systemd stop
	 //signal(SIGPIPE, SignalPIPEHandler);// Error trying to write/read to a socket
	 //signal(SIGSEGV, SignalSegmentationFaultHandler);// Segmentation fault
 
	 if ( argc == 1 ) {
	 	cout << "No arguments were passed!" << endl;
	 }
	 else{
		 //cout << "Arguments" << endl;
		 //for (int i = 1; i < argc; ++i ) {
		 //	printf( "  %d. %s\n", i, argv[i] );
		 //}
	 	GPIOagent.initialDesiredDCvoltage=stof(argv[1]);
	 	GPIOagent.currentVoltageValue=GPIOagent.initialDesiredDCvoltage;
	 	GPIOagent.TARGET_CPS=stod(argv[2]);
	 }
	 
	 cout << "GPIOspadSYScont started..." << endl;
	 
	 GPIOagent.m_start(); // Initial status
	 
	 bool isValidWhileLoop=true;
	 if (GPIOagent.getState()==GPIO::APPLICATION_EXIT){isValidWhileLoop = false;}
	 else{isValidWhileLoop = true;}
	 
	 //CKPDagent.GenerateSynchClockPRU();// Launch the generation of the clock
	 // First initial volage bias up
	 GPIOagent.SPIrampVoltage(GPIOagent.spi_fd, GPIOagent.initialDesiredDCvoltage, 2.0, true);
	 GPIOagent.SendControlSignals();

	 // Set initial Time Wall
	GPIOagent.SetNewFutureTimeWall();
	 
	 while(isValidWhileLoop && !signalReceivedFlag.load()){ 
	 	//CKPDagent.acquire();
	   //try{
	 	//try {
	    // Code that might throw an exception 
	 	// Check if there are need messages or actions to be done by the node
	 	
	       switch(GPIOagent.getState()) {
	           case GPIO::APPLICATION_RUNNING: {               
	               // Do Some Work
	               GPIOagent.HandleInterruptPRUsActive();
	               break;
	           }
	           case GPIO::APPLICATION_PAUSED: { // When no corrections are aimed (maybe because they are true signal detections periods)
	               // Maybe do some checks if necessary 
	           		GPIOagent.HandleInterruptPRUsPaused();
	               break;
	           }
	           case GPIO::APPLICATION_EXIT: {                  
	               isValidWhileLoop=false;//break;
	           }
	           default: {
	               // ErrorHandling Throw An Exception Etc.
	           }
	        } // switch
	        //cout << "Ctrl+x pressed!" << endl;           	
	       	if (read(STDIN_FILENO, &KeyboardC, 1) == 1) {
	            // Pressed key handling
	            if (GPIOagent.getState() == GPIO::APPLICATION_PAUSED){
	            	GPIOagent.m_resume();
	            	cout << "System resumed. Press any key to pause..." << endl;
	            	GPIOagent.SetNewFutureTimeWall();
	            }
	            else{
	            	GPIOagent.m_pause();
	            	//cout << "System paused. Press any key to resume..." << endl;
	            }
	        }
	        
	        if (GPIOagent.getState() == GPIO::APPLICATION_PAUSED){
	        	cout << "System paused. Press any key to resume..." << endl;
			}
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

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore

 return 0; // Everything Ok
}
