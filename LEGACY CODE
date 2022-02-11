.include "m128def.inc"
.equ CRC_GEN = 0x35 ;110101, x^5+x^4+x^2+1
.equ STACK_END = 0x10EB
.equ STACK_START = 0x10FF
.equ RAM_END = 0x10FF
.def PACKET_IN = R20
.def PACKET_OUT = R21
.def CRC_CHECK = R22
.org 0x00

.cseg

.macro crc3
	mov r17, @0
	ldi r18, CRC_GEN
	lsl r18
	lsl r18 ;shift crc generator to align with packet
	sbrc r17, 7 ;check msb of data
	eor r17, r18 ;if msb is 1, xor 
	lsl r17 ;shift data
	; repeat twice
	sbrc r17, 7 
	eor r17, r18 
	lsl r17 

	sbrc r17, 7
	eor r17, r18 

	lsr r17
	lsr r17 ;shift data back to allign with crc

	or @0, r17
.endmacro

PRE_INIT:
	ldi r16, 0xFF
	out ddrb, r16 ;Readout LEDs
	out ddre, r16 ; Packet-out LEDs
	ldi r16, 0x00
	sts porte, r16
	sts ddrf, r16 ;Packet-In
	out ddrd, r16 ;Receive button and control signals
	sbi ddrd, 0 ;Ready LED
	ldi r16, 0x80
	ldi xl, 0x00
	ldi xh, 0x01
	STACK_INIT:
		ldi r16, high(STACK_START)
		out sph, r16
		ldi r16, low(STACK_START)
		out spl, r16
	call INIT
	push PACKET_OUT
PRE_SERVICE:
	call SERVICE_READOUT
	sbis pind, 3 ;check if power is activated
	jmp STACK_INIT ;if power is off, check again
	sbi portd, 0 ;light Ready LED
	sbis pind, 7 ;check if receive button is pressed
	jmp PRE_SERVICE; if not, check again
RECEIVE_CHECK:
	SBIC pind, 7; check if receieve is off
	jmp RECEIVE_CHECK ;if not, check again
	cbi portd, 0 ;Ready -> OFF
	call CAPTURE
	SBRC PACKET_IN, 7 ;check if packet is command
	JMP DATA
	JMP COMMAND
DATA:
	pop r16
	push r16
	cpi r16, 0x00 ;check if the stack is empty
	breq PUSH_PACKET ;if so, push packet
	pop r16 ;if not, pop from the stack
	PUSH_PACKET:
		push PACKET_IN ;push data packet onto stack
		JMP PRE_SERVICE
COMMAND:
	pop r16
	push r16
	sbrc r16, 7 ;check if there is a data packet in memory
	JMP DATA_EXIST
	JMP NO_DATA
DATA_EXIST: ;if there is a data packet in stack, run crc11 and check request type
	pop r19 ;load data from stack
	mov zh, r19 ;load data byte from stack
	CALL CRC_CHECK11 ;check data and command for crc
	ldi r16, 0x00
	CP CRC_CHECK, r16 ;check if crc failed
	BREQ FAILED_11
	JMP PASS_11
	FAILED_11:
		pop r16
		CALL REPEAT_REQUEST
		JMP PRE_SERVICE
	PASS_11:
		mov r17, PACKET_IN
		andi r17, 0x60 ; check if log request
		cpi r17, 0x20
		brne PRE_SERVICE ;if not, go back
		mov r16, r19 
		jmp LOG

NO_DATA: ;if there is no data packet in the stack, run crc3 and check command type
	CALL CRC_CHECK3 ;run crc check for command
	ldi r16, 0x00
	cp CRC_CHECK, r16 ;check if it passed
	BREQ FAIL
	PASS:
		mov r17, PACKET_IN
		andi r17, 0x60
		cpi r17, 0x40 ; check if command is acknowledge signal
		BREQ POP_STACK ;if yes, pop stack and return
		jmp CHECK_REPEAT ;if no, check if it has a repeat signal
		CHECK_REPEAT:
		mov r17, PACKET_IN
		andi r17, 0x60
		cpi r17, 0x60 ;check if command is repeat signal
		brne RELATIVE_SKIP ;if not, return
		pop r16 ;if yes, peep stack
		push r16
		cpi r16, 0x00
		breq RELATIVE_SKIP ;if empty, return
		JMP TOS_TO_SENS ;if not empty, transmit top of stack
		RELATIVE_SKIP:
			jmp PRE_SERVICE
	FAIL:
		CALL REPEAT_REQUEST
		JMP PRE_SERVICE

	POP_STACK:
		pop r16
		jmp PRE_SERVICE

TOS_TO_SENS: ;pops top of stack and transmits it to sensors
	pop PACKET_OUT
	CALL TRANSMIT
	JMP PRE_SERVICE


INIT:
	ldi PACKET_OUT, 0x00 ;Reset Request
	CRC3 PACKET_OUT ;CRC calculation macro
	CALL TRANSMIT
	RET	
LOG:
	cpi xh, high(STACK_END) ;check if attempting to overwrite stack
	brne EXT_MEM
	cpi xl, low(STACK_END)
	brne EXT_MEM
	ldi xh, high(0x10)
	ldi xl, low(0x00)
	jmp WRITE

	EXT_MEM:
		cpi xh, high(RAM_END) ;check if end of memory
		brne WRITE
		cpi xl, low(RAM_END)
		brne WRITE
		ldi xh, 0x01
		ldi xl, 0x00 ;reset memory
	WRITE:
		ST x+, r16 ;write to memory
		ldi r16, 0x40 ;acknowledge signal
		crc3 r16 ;create crc code
		push r16 ;push onto stack
		jmp TOS_TO_SENS
REPEAT_REQUEST:
	ldi PACKET_OUT, 0x60 ;load repeat request
	crc3 PACKET_OUT ;encode
	CALL TRANSMIT ;transmit
	RET

CRC_CHECK3:
	mov r9, PACKET_IN
	crc3 r9
	cp r9, PACKET_IN
	BREQ PASSED
	ldi CRC_CHECK, 0x00 ; if crc check failed, set to 0
	RET
	PASSED:
		ldi CRC_CHECK, 0xFF ; if crc check succeeds, set to 1s
		RET

CRC_CHECK11:
	mov	zl, PACKET_IN
	ANDI zl, 0xE0 ;mask the crc code from lower bit, 0xe0 = 0b11100000
	ldi r16, 12 ;Counter
	ldi r18, CRC_GEN
	lsl r18
	lsl r18 ;shift crc generator to align with packet
	S1:
		DEC r16
		breq CHECK ;if coutner is 0, break
		sbrc zh, 7 ;check msb of data, if 1 then xor, else shift
		jmp S2
		lsl zl
		rol zh
		jmp S1
		S2:
			eor zh,r18
			lsl zl
			rol zh
			jmp S1
	CHECK:
		mov r16, PACKET_IN
		lsl r16
		lsl r16
		lsl r16 ;shift packet 3 times to allign with crc
		cp r16, zh ;compare crc with the one calculated
		breq PASSED_11
		ldi CRC_CHECK, 0x00 ; if crc check failed, set to 0
		RET
	PASSED_11:
		ldi CRC_CHECK, 0xFF ; if crc check succeeds, set to 1s
		RET

TRANSMIT:
	out porte, PACKET_OUT
	RET
CAPTURE:
	cbi portd, 0 ;turn off Ready LED
	in PACKET_IN, pinf
	call DELAY
	RET

SERVICE_READOUT:
	sbic pind, 4 ;check memdump pin
	call MEMDUMP
	sbic pind, 5; check last entry pin
	CALL LASTENTRY
	RET

	MEMDUMP:
		cpi xh, high(STACK_START) ;check if attempting to read stack
		brne CHECK_EXT_MEM
		cpi xl, low(STACK_START)
		brne CHECK_EXT_MEM
		ldi xh, high(STACK_END+1)
		ldi xl, low(STACK_END+1)
		jmp DUMP

		CHECK_EXT_MEM:
			cpi xh, high(0x0100) ;check if at start of memory
			brne DUMP
			cpi xl, low(0x0100)
			brne DUMP
			RET ;end of dump
		DUMP:
			LD r6, -x
			out portb, r6
			call DELAY
			sbic pind, 4 ;check if mem dump pin is off
			jmp MEMDUMP
			RET
	LASTENTRY:
		ld r6, -x
		adiw x, 1
		out portb, r6
		call DELAY
		ldi r16, 0x00
		out portb, r16
		RET
DELAY:
ldi r16, 60
	loop1:
		dec r16 ; 1c
		ldi r17, 0xFF ; 1c
		loop2:
			dec r17
			ldi r18, 0x0F
			loop3:
			   dec r18
			   cpse r18, r1
			   rjmp loop3
			cpse r17, r1
			rjmp loop2
		cpse r16, r1 ; 1c
		rjmp loop1 ; 1c
	ret
