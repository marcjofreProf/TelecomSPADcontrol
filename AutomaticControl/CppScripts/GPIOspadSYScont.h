/* Author: Prof. Marc Jofre
Dept. Network Engineering
Universitat Politècnica de Catalunya - Technical University of Catalonia

Modified: 2025
Created: 2024

Header declaration file for handling PRU

*/

#ifndef GPIO_H_
#define GPIO_H_
#include<string>
#include<fstream>
#include<cmath>// abs, fmod, fmodl, floor, ceil
// Threading
#include <thread>
// Semaphore
#include <atomic>
// Time/synchronization management
#include <chrono>
// Timer management
#include <sys/timerfd.h>
#include <sys/select.h>
// SPIc ommunications
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

using namespace std;

using std::string;
using std::ofstream;
using std::ifstream;
using std::fstream;

#define WaitTimeAfterMainWhileLoop 500000000 //nanoseconds. Maximum 999999999
#define PRUclockStepPeriodNanoseconds		5.00000 //4.99999 // Very critical parameter experimentally assessed. PRU clock cycle time in nanoseconds. Specs says 5ns, but maybe more realistic is the 24 MHz clock is a bit higher and then multiplied by 8

namespace exploringBB {

	typedef int (*CallbackType)(int);
	enum GPIO_DIRECTION{ INPUT, OUTPUT };
	enum GPIO_VALUE{ LOW=0, HIGH=1 };
	enum GPIO_EDGE{ NONE, RISING, FALLING, BOTH };

	class GPIO {
public: //Variables
	enum ApplicationState { // State of the agent sequences
		APPLICATION_RUNNING = 0,
		APPLICATION_PAUSED = 1,  // Out of Focus or Paused If In A Timed Situation
		APPLICATION_EXIT = -1,
	    };
	// SPI communications
	int spi_fd; // SPI file descriptor

private:// Variables
	ApplicationState m_state;
	// Priority values
	int PriorityValRegular=60; // Regular priority during most of the operation
	int PriorityValTop=70; // Top priority for critical operations
	// Semaphore
	unsigned long long int UnTrapSemaphoreValueMaxCounter=1000;//MAx counter trying to acquire semaphore, then force release
	std::atomic<bool> valueSemaphore{true};// Start as 1 (open or acquireable)
	//std::thread threadRefSynch; // Process thread that executes requests/petitions without blocking
	// Time/synchronization management
	struct my_clockChrono
	{
		using duration   = std::chrono::nanoseconds;
		using rep        = duration::rep;
		using period     = duration::period;
		using time_point = std::chrono::time_point<my_clockChrono>;
	    static constexpr bool is_steady = false;// true, false.

	    static time_point now()
	    {
	    	timespec ts;
			if (clock_gettime(CLOCK_MONOTONIC, &ts))// CLOCK_MONOTONIC: since we want to actuate at the same interval lengths // CLOCK_REALTIME//CLOCK_TAI. Seems that CLOCK_TAI does not work with timerfd
				throw 1;
			using sec = std::chrono::seconds;
			return time_point{sec{ts.tv_sec}+duration{ts.tv_nsec}};
		}
	};

	int WaitTimeInterruptPRU0=7500000; //up to 20000000 with Simple TTG. In microseconds
	int WaitTimeInterruptPRU1=7500000; // In microseconds. 

	using ClockChrono = my_clockChrono;//Clock = std::chrono::system_clock;// Since we use a time sleep, it might make sense a system_clock//tai_clock, system_clock or steady_clock;
	using TimePointChrono = std::chrono::time_point<ClockChrono>;
	TimePointChrono TimePointClockCurrentSynchPRU0future=std::chrono::time_point<ClockChrono>();// For synch purposes
	struct timespec requestTimeWait;
	// PRU
	static void *ddrMem, *sharedMem, *pru0dataMem, *pru1dataMem;
	static void *pru_int;       // Points to start of PRU memory.
	//static int chunk;
	static unsigned int *sharedMem_int,*pru0dataMem_int,*pru1dataMem_int;
	
	// SHARED RAM to file dump	
	unsigned int* CalpHolder; // 32 bits
	unsigned int* valp; // 32 bits

	// SPI communication
	// Voltage to SPI conversion constants
    const float MIN_V = 39.5;
    const float MAX_V = 88.7;
    const float RATIO = (MAX_V - MIN_V) / 255.0;
	float currentSPIvalue=MIN_V; // In volts // Initial value and follow up values storage
	// Detection counters
	unsigned int DetCounterCh[4]; // Holder of the detections per channel

public:	// Functions/Methods
	// PRU
	GPIO(); // initializates PRU operation
	// PRU synchronization
	int NonBusyTimeWall();
	// Managing status of this Agent
	// resume may keep track of time if the application uses a timer.
    // This is what makes it different than start() where the timer
    // in start() would be initialized to 0. And the last time before
    // paused was trigger would be saved, and then reset as new starting
    // time for your timer or counter. 
    ApplicationState getState() const { return m_state; }    
    bool m_start() { m_state = APPLICATION_RUNNING; return true; }
    bool m_pause() { m_state = APPLICATION_PAUSED; return true; }    
    bool m_resume() { m_state = APPLICATION_RUNNING; return true; }      
    bool m_exit() { m_state = APPLICATION_EXIT;  return false; }
	int InitAgentProcess();
	int SendControlSignals(); // Write the AC Geiger signals (Frequency, duty cycle...)Uses output pins to clock subsystems physically generating qubits or entangled qubits
	int LOCAL_DDMinit();	
	int DisablePRUs();
	int HandleInterruptPRUs(); // Main call function to manage the operation in/out of the PRUs
	int RelativeNanoSleepWait(unsigned int TimeNanoSecondsSleep);
	~GPIO();  //destructor
	// For SPI communications
	int SPIrampVoltage(int spi_fdAux, float desired_voltage, float max_rate, bool verbose);

private: // Functions/Methods
	int KillcodePRUs();
	// Task manager priority
	bool setMaxRrPriority(int PriorityValAux);
	// Sempahore
	void acquire();
	void release();		
	// Data processing
	int DDRdumpdata();
	int ReadTimeCounts();// Read the associated SPAD counters
	// Mean filter
	long double LongDoubleMeanFilterSubArray(long double* ArrayHolderAux,int MeanFilterFactor);
	int IntMeanFilterSubArray(int* ArrayHolderAux,int MeanFilterFactor);
	double DoubleMeanFilterSubArray(double* ArrayHolderAux,int MeanFilterFactor);
	long long int LLIMeanFilterSubArray(long long int* ArrayHolderAux,int MeanFilterFactor);
	// Median filter
	long long int LLIMedianFilterSubArray(long long int* ArrayHolderAux,int MedianFilterFactor);
	long double LongDoubleMedianFilterSubArray(long double* ArrayHolderAux,int MedianFilterFactor);
	double DoubleMedianFilterSubArray(double* ArrayHolderAux,int MedianFilterFactor);
	int IntMedianFilterSubArray(int* ArrayHolderAux,int MedianFilterFactor);	
	int LongDoubleBubbleSort(long double* arr,int MedianFilterFactor);
	int DoubleBubbleSort(double* arr,int MedianFilterFactor);
	int IntBubbleSort(int* arr,int MedianFilterFactor);
	// SPI communications
	uint8_t spiTransferByte(int spi_fdAux, uint8_t tx);
};

} /* namespace exploringBB */

#endif /* GPIO_H_ */
