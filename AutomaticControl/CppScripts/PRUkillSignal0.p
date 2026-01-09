.origin 0				// start of program in PRU memory
.entrypoint EXIT			// program entry point (for debbuger)

#define CONST_PRUCFG         C4

INITIATIONS:
	LBCO	r0, CONST_PRUCFG, 4, 4 // Enable OCP master port
	// OCP master port is the protocol to enable communication between the PRUs and the host processor
	CLR		r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	SBCO	r0, CONST_PRUCFG, 4, 4
WRITEOFF:
	MOV		r30.b0, 0x00 // All off
EXIT:
	HALT
ERR:	// Signal error
	HALT
