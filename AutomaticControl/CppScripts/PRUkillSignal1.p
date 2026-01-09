.origin 0				// start of program in PRU memory
.entrypoint WRITEOFF			// program entry point (for debbuger)

WRITEOFF:
	MOV		r30.b0, 0x00 // All off
EXIT:
	HALT
ERR:	// Signal error
	HALT
