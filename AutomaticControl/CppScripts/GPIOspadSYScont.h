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

#define WaitTimeAfterMainWhileLoop 990000000 //nanoseconds. Maximum 999999999
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
	

private:// Variables
	ApplicationState m_state;
	// Priority values
	int PriorityValRegular=60; // Regular priority during most of the operation
	int PriorityValTop=70; // Top priority for critical operations
	// Semaphore
	unsigned long long int UnTrapSemaphoreValueMaxCounter=1000;//MAx counter trying to acquire semaphore, then force release
	int whileProtAuxMax=10000; // Value of protecton against blocking indefenitely, which produces the node to go down (broken pipe and re-start)
	int whileProtAux=whileProtAuxMax; // Protection againts blocking indefinetely
	std::atomic<bool> valueSemaphore{true};// Start as 1 (open or acquireable)
	std::atomic<bool> ManualSemaphore{false};
	std::atomic<bool> ManualSemaphoreExtra{false};
	std::thread threadRefSynch; // Process thread that executes requests/petitions without blocking
	// Time/synchronization management
	struct my_clock
	{
		using duration   = std::chrono::nanoseconds;
		using rep        = duration::rep;
		using period     = duration::period;
		using time_point = std::chrono::time_point<my_clock>;
	    static constexpr bool is_steady = false;// true, false.

	    static time_point now()
	    {
	    	timespec ts;
			if (clock_gettime(CLOCK_REALTIME, &ts))// CLOCK_REALTIME//CLOCK_TAI. Seems that CLOCK_TAI does not work with timerfd
				throw 1;
			using sec = std::chrono::seconds;
			return time_point{sec{ts.tv_sec}+duration{ts.tv_nsec}};
		}
	};

	int WaitTimeInterruptPRU0=7500000; //up to 20000000 with Simple TTG. In microseconds
	int WaitTimeInterruptPRU1=7500000; // In microseconds. 

	unsigned long long int TimePRU1synchPeriod=100000000; // In nanoseconds and multiple of PRUclockStepPeriodNanoseconds// The faster the more corrections, and less time passed since last correction, but more averaging needed. Also, there is a limit on the lower limit to procees and handle interrupts. Also, the sorter the more error in the correct estimation, since there has not elapsed enough time to compute a tendency (it also happens with PRUdetCorrRelFreq() method whre a separation TagsSeparationDetRelFreq is inserted). The limit might be the error at each iteration, if the error becomes too small, then it cannot be corrected. Anyway, with a better hardware clock (more stable) the correctioons can be done more separated in time).
	using Clock = my_clock;//Clock = std::chrono::system_clock;// Since we use a time sleep, it might make sense a system_clock//tai_clock, system_clock or steady_clock;
	using TimePoint = std::chrono::time_point<Clock>;
	TimePoint QPLAFutureTimePoint=std::chrono::time_point<Clock>();// For matching trigger signals and timetagging
	TimePoint TimePointClockCurrentSynchPRU1future=std::chrono::time_point<Clock>();// For synch purposes
	unsigned long TimePRUcommandDelay=250000;//250000;// In nanoseconds. If too large, it disastabilizes the timming performance. Very important parameter!!! When duration_FinalInitialMeasTrigAuxAvg properly set then is around 4000
	int tfd; // Timer. Attention: // close the time descriptor in the destructor
	fd_set rfds;
	struct timeval TimerTimeout;
	int duration_FinalInitialMeasTrigAuxAvg=0;
	// PRU
	static int mem_fd;
	static void *ddrMem, *sharedMem, *pru0dataMem, *pru1dataMem;
	static void *pru_int;       // Points to start of PRU memory.
	//static int chunk;
	static unsigned int *sharedMem_int,*pru0dataMem_int,*pru1dataMem_int;
	
	// SHARED RAM to file dump
	int iIterDump;
	int CurrentiIterDump;
	unsigned int NumSynchPulses=0;
	unsigned short* valpHolder;
	//unsigned short* valpAuxHolder;
	unsigned int* CalpHolder; // 32 bits
	unsigned short* valp; // 16 bits
	//unsigned short* valpAux; // 16 bits
	unsigned int valCycleCountPRU=0; // 32 bits // Made relative to each acquisition run
	//unsigned int valIEPtimerFinalCounts; // 32 bits
	unsigned long long int extendedCounterPRU=0; // 64 bits
	unsigned long long int extendedCounterPRUholder=0; // 64 bits.
	unsigned long long int extendedCounterPRUholderOld=0; // 64 bits
	unsigned long long int extendedCounterPRUaux=0; // 64 bits

	// SPI communication
	// Voltage to SPI conversion constants
    const float MIN_V = 39.5;
    const float MAX_V = 88.7;
    const float RATIO = (MAX_V - MIN_V) / 255.0;
      // SPI communications
	int spi_fd; // SPI file descriptor
	float currentSPIvalue=MIN_V; // In volts // Initial value and follow up values storage

public:	// Functions/Methods
	// PRU
	GPIO(); // initializates PRU operation
	// Managing status of this Agent
    ApplicationState getState() const { return m_state; }	
    bool m_start() { m_state = APPLICATION_RUNNING; return true; }
    bool m_pause() { m_state = APPLICATION_PAUSED; return true; } 
    // resume may keep track of time if the application uses a timer.
    // This is what makes it different than start() where the timer
    // in start() would be initialized to 0. And the last time before
    // paused was trigger would be saved, and then reset as new starting
    // time for your timer or counter. 
    bool m_resume() { m_state = APPLICATION_RUNNING; return true; }      
    bool m_exit() { m_state = APPLICATION_EXIT;  return false; }
	int InitAgentProcess();
	int LOCAL_DDMinit();	
	int DisablePRUs();
	int HandleInterruptPRUs(); // Main call function to manage the operation in/out of the PRUs
	int RelativeNanoSleepWait(unsigned int TimeNanoSecondsSleep);
	~GPIO();  //destructor
	// For SPI communications
	int SPIrampVoltage(float desired_voltage, float max_rate, bool verbose);

private: // Functions/Methods
	int KillcodePRUs();
	// Task manager priority
	bool setMaxRrPriority(int PriorityValAux);
	// Sempahore
	void acquire();
	void release();
	// PRU synchronization
	struct timespec SetWhileWait();	
	// Data processing
	int DDRdumpdata();
	int ReadTimeCounts();// Read the associated SPAD counters
	int SendControlSignals(); // Write the AC Geiger signals (Frequency, duty cycle...)Uses output pins to clock subsystems physically generating qubits or entangled qubits
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
	uint8_t spiTransferByte(uint8_t tx);
};

} /* namespace exploringBB */

#endif /* GPIO_H_ */
