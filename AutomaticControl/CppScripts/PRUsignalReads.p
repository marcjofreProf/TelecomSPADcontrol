// PRUsignalReads.p
// Time Tagging functionality on PRU0 with Shared Memory Access (not Direct Memory Access nor DDR)
// It is pending studying if it is worth making use of DMA or DDR in terms of larger memory space or transfer speed

.origin 0
.entrypoint INITIATIONS

#include "PRUassReadsScript.hp"

#define LASTSHAREDRAM 11996 //12000-4 // Address of the last position of the shared RAM
// Length of acquisition:
#define EXITCOUNTER 0xFFFFFFFF // maximum number of caiunts of all combined counter registers

// *** LED routines, so that LED USR0 can be used for some simple debugging
// *** Affects: r28, r29. Each PRU has its of 32 registers
.macro LED_OFF
	MOV	r28, 1<<21
	MOV	r29, GPIO1_BANK | GPIO_CLEARDATAOUToffset
	SBBO	r28, r29, 0, 4
.endm

.macro LED_ON
	MOV	r28, 1<<21
	MOV	r29, GPIO1_BANK | GPIO_SETDATAOUToffset
	SBBO	r28, r29, 0, 4
.endm

//Notice:
// - xBCO instructions when using constant table pointers. Faster, probably 3 clock cycles
// - xBBO instructions when using directly registers. Slower, probably between 3 and 5 clock cycles

// r0 is arbitrary used for operations
// r1 is reserved for the detection mask
// r2 reserved mapping control register

// r3 individual channel counter 1
// r4 individual channel counter 2
// r5 individual channel counter 3
// r6 individual channel counter 4

// r7 reserved for 0 value (zeroing registers)
// r8 reserved for pins detection
// r9 reserved for reading DWT_CYCCNT

// r10 is arbitrary used for operations

//// If using the cycle counte rin the PRU (not adjusted to synchronization protocols)
// We cannot use Constan table pointers since the base addresses are too far
// r12 reserved for 0x22000 Control register
// r13 reserved for 0x2200C DWT_CYCCNT

// r20 reserved for exit counter

//// If using IET timer (potentially adjusted to synchronization protocols)
// We can use Constant table pointers C26
// CONST_IETREG 0x0002E000
// IET Count 0xC offset

// r28 is mainly used for LED indicators operations
// r29 is mainly used for LED indicators operations
// r30 is reserved for output pins
// r31 is reserved for inputs pin
INITIATIONS:// This is only run once
	LBCO	r0, CONST_PRUCFG, 4, 4 // Enable OCP master port
	// OCP master port is the protocol to enable communication between the PRUs and the host processor
	CLR		r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	SBCO	r0, CONST_PRUCFG, 4, 4
    
	// Configure the programmable pointer register for PRU by setting c24_pointer[3:0] // related to pru data RAM, where the commands will be found
	// This will make C24 point to 0x00000000 (PRU data RAM).
	MOV		r0, OWN_RAM | OWN_RAMoffset// | OWN_RAMoffset // When using assembler, the PRU does not put data in the first addresses of OWN_RAM (when using c++ PRU direct programming the PRU  might use some initial addresses of OWN_RAM space
	MOV		r10, 0x24000+0x20// | C24add//CONST_PRUDRAM
	SBBO	r0, r10, 0, 4//SBCO	r0, CONST_PRUDRAM, 0, 4  // Load the base address of PRU0 Data RAM into C24
	
//	// This will make C26 point to 0x0002E000 (IEP).
//	MOV	r0, 0x0002E000//
//	SBCO	r0, CONST_IETREG, 0, 4  // Load the base address of IEP

	// Configure the programmable pointer register for PRU by setting c28_pointer[15:0] // related to shared RAM
	// This will make C28 point to 0x00010000 (PRU shared RAM).
	// http://www.embedded-things.com/bbb/understanding-bbb-pru-shared-memory-access/	
	MOV		r0, SHARED_RAM // 0x100                  // Set C28 to point to shared RAM
	MOV		r10, 0x22000+0x28//PRU0_CTRL | C28add //CONST_PRUSHAREDRAM
	SBBO 	r0, r10, 0, 4//SBCO	r0, CONST_PRUSHAREDRAM, 0, 4 //SBBO r0, r10, 0, 4
		
//	// Configure the programmable pointer register for PRU by setting c31_pointer[15:0] // related to ddr.
//	// This will make C31 point to 0x80001000 (DDR memory). 0x80000000 is where DDR starts, but we leave some offset (0x00001000) to avoid conflicts with other critical data present
//	https://groups.google.com/g/beagleboard/c/ukEEblzz9Gk
//	MOV	r0, DDR_MEM                    // Set C31 to point to ddr
//	//MOV	r10, PRU0_CTRL | C31add
//	SBCO    r0, CONST_DDR, 0, 4

	//Load values from external DDR Memory into Registers R0/R1/R2
	//LBCO      r0, CONST_DDR, 0, 12
	//Store values from read from the DDR memory into PRU shared RAM
	//SBCO      r0, CONST_PRUSHAREDRAM, 0, 12

//      LED_ON	// just for signaling initiations
//	LED_OFF	// just for signaling initiations
	// Using DWT cycle counter
	MOV	r12, 0x22000
	MOV	r13, 0x2200C // Actual value of the DWT_CYCNT. Maybe it can be referenced from CONST_PRUCFG | 0xC

	// Initializations - some, just in case
	LDI		r9, 0 // initialize to 0
	MOV		r1, 0x0000C0A0 // Detection mask
	LDI		r7, 0 // Register for clearing other registers
	MOV 	r20, EXITCOUNTER // Maximum value to start with to exit if nothing happens

	// Initial Re-initialization of DWT_CYCCNT
	LBBO	r2.b0, r12, 0, 1 // r2 maps b0 control register
	CLR		r2.t3
	SBBO	r2.b0, r12, 0, 1 // stops DWT_CYCCNT
	SBBO	r7, r13, 0, 4 // reset DWT_CYCNT
//	LBBO	r2.b0, r12, 0, 1 // r2 maps b0 control register
//	SET		r2.t3
	//SBBO	r2.b0, r12, 0, 1 // Enables DWT_CYCCNT. We start it when the commands enters
		
	// Initial Re-initialization for IET counter. Used in the other PRU1
	// The Clock gating Register controls the state of Clock Management
//	//LBCO 	r0, CONST_PRUCFG, 0x10, 4                    
//	MOV 	r0, 0x24924
//	SBCO 	r0, CONST_PRUCFG, 0x10, 4 
//	//LBCO	r2, CONST_IETREG, 0, 1 //
//	//SET ocp_clk:1 or of iep_clk:0
//	MOV	r0, 0
//	SBCO 	r0, CONST_PRUCFG, 0x30, 4
//	// IEP configuration
//	MOV	r0, 0x111 // Enable and Define increment value to 1
//	SBCO	r0, CONST_IETREG, 0, 4 // Enables IET count and sets configuration
//	// Deactivate IEP compensation
//	SBCO 	r7, CONST_IETREG, 0x08, 4
	
CMDLOOP:
	QBBC	CMDLOOP, r31, 30	// Reception or not of the host interrupt
	SBCO	r7.b0, C0, 0x24, 1 // Reset host interrupt	
CMDLOOP2:// Double verification of host sending start command
	LBCO	r0.b0, CONST_PRUDRAM, 0, 1 // Load to r0 the content of CONST_PRUDRAM with offset 0, and 1 bytes. It is the command to start
	QBEQ	CMDLOOP, r0.b0, 0 // loop until we get an instruction
	SBCO	r7.b0, CONST_PRUDRAM, 0, 1 // Store a 0 in CONST_PRUDRAM with offset 0, and 1 bytes. Reset the command to start
	//MOV 	r31.b0, PRU0_ARM_INTERRUPT+16// Here send interrupt to host to measure time
RESETCOUNTS: // Reset counters
	LDI		r3, 0
	LDI		r4, 0
	LDI		r5, 0
	LDI		r6, 0
	LDI		r9, 0 // make sure to reset this value
	MOV 	r20, EXITCOUNTER // Maximum value to start with to exit 
DWTSTART:
	// Re-start DWT_CYCNT
	LBBO	r2, r12, 0, 1 // r2 maps b0 control register
//	CLR		r2.t3
//	SBBO	r2, r12, 0, 1 // stops DWT_CYCCNT
//	SBCO	r7, CONST_DWTCYCNT, 0, 4 // reset DWT_CYCNT
	SET		r2.t3
	SBBO	r2.b0, r12, 0, 1 // Enables DWT_CYCCNT
WAIT_FOR_EVENT: // At least dark counts will be detected so detections will happen
	// Check if the timer is done
	LBBO	r9, r13, 0+2, 1 // Read the relevant byte value of DWT_CYCNT.
	QBLE    FINISH, r9, 128 //Timer done. Equivalent to 40 ms
	// Then measure what should be 1 (for edge detection)	
	MOV		r8, r31 // Consecutive red for edge detection
	AND		r8, r8, r1 // Mask to make sure there are no other info

	MOV		r8, r31 // Dummy

CHECKDET:		
	QBEQ 	WAIT_FOR_EVENT, r8, 0//QBEQ 	WAIT_FOR_EVENT, r6.w0, 0 //all the b0 above can be converted to w0 to capture more channels, but then in the channel tag recorded has to be increaed and appropiatelly handled in c++ (also the number of tags per run has to be reduced)
	// If the program reaches this point, at least one of the bits is high

	MOV		r8, r1 // Dummy

COUNTERS:
	SUB 	r20, r20, 1 // Substract 1 to the exit counter
	LDI		r10, 0 // Clear value of r10
	LSR		r8, r8, 5 // Move relevant bits to the right
	AND		r10, r8, 1 // Move relevant bit to r10
	ADD		r3, r3, r10 // Increase counter 1 by r10
	LSR		r8, r8, 2 // Move relevant bits to the right
	AND		r10, r8, 1 // Move relevant bit to r10
	ADD		r4, r4, r10 // Increase counter 2 by r10
	LSR		r8, r8, 7 // Move relevant bits to the right
	AND		r10, r8, 1 // Move relevant bit to r10
	ADD		r5, r5, r10 // Increase counter 3 by r10
	LSR		r8, r8, 1 // Move relevant bits to the right
	AND		r10, r8, 1 // Move relevant bit to r10
	ADD		r6, r6, r10 // Increase counter 4 by r10
	// Check to see if we still need to read more data due to DWT_CYCNT
	QBNE 	WAIT_FOR_EVENT, r20, 0 // loop if we've not finished
FINISH:
	// Faster Concatenated Checks writting	
	SET     r30.t11	// enable the data bus. it may be necessary to disable the bus to one peripheral while another is in use to prevent conflicts or manage bandwidth.
	////////////////////////////////////////	
	// Indicate number of captures
	SBCO 	r3, CONST_PRUSHAREDRAM, 0, 4
	SBCO 	r4, CONST_PRUSHAREDRAM, 4, 4
	SBCO 	r5, CONST_PRUSHAREDRAM, 8, 4
	SBCO 	r6, CONST_PRUSHAREDRAM, 12, 4
	// STOP DWT_CYCNT
	LBBO	r2.b0, r12, 0, 1 // r2 maps b0 control register
	CLR		r2.t3
	SBBO	r2.b0, r12, 0, 1 // stops DWT_CYCCNT
	SBBO	r7, r13, 0, 4 // reset DWT_CYCNT
	//LED_ON // For signaling the end visually and also to give time to put the command in the OWN-RAM memory
	//LED_OFF
	MOV		r31.b0, PRU0_ARM_INTERRUPT+16// Notification sent at the beginning of the signal//SBCO 	r17.b0, CONST_PRUDRAM, 4, 1 // Put contents of r0 into CONST_PRUDRAM// code 1 means that we have finished. This can be substituted by an interrupt: MOV 	r31.b0, PRU0_ARM_INTERRUPT+16
	JMP 	CMDLOOP // finished, wait for next command. So it continuosly loops	
EXIT:
	SET     r30.t11	// enable the data bus. it may be necessary to disable the bus to one peripheral while another is in use to prevent conflicts or manage bandwidth.
	HALT // Halt the processor
ERR:
	SET     r30.t11	// enable the data bus. it may be necessary to disable the bus to one peripheral while another is in use to prevent conflicts or manage bandwidth.
	LED_ON
//	JMP INITIATIONS
//	JMP ERR
	HALT