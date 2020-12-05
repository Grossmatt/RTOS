;asm.s



	.def setPSP
	.def setASP
	;.def waitMicrosecond
	;.def WMS_LOOP0
	;.def WMS_LOOP1
	;.def WMS_DONE0
	;.def WMS_DONE1
	.def getPSP
	.def getMSP
	.def pushR4toR11toPSP
	.def popR4toR11fromPSP
	.def pushPSP
	.def getSVCnumAdd
	.def getRfromPSP



.thumb
.const



.text

setPSP:
		MSR PSP, R0
		ISB
		BX LR

setASP:
		MRS R0, CONTROL
		ORR R0, R0, #2
		MSR CONTROL, R0
		ISB
		BX LR

;waitMicrosecond:
;WMS_LOOP0:   		MOV  R1, #6
;WMS_LOOP1:   		SUB  R1, #1
;             		CBZ  R1, WMS_DONE1
;           			NOP
;             		NOP
;            		B    WMS_LOOP1
;WMS_DONE1:   		SUB  R0, #1
;             		CBZ  R0, WMS_DONE0
;            		NOP
;             		B    WMS_LOOP0
;WMS_DONE0:
;					BX LR

getPSP:
		MRS R0, PSP
		BX LR


getMSP:
		MRS R0, MSP
		BX LR

pushR4toR11toPSP:
		MRS R0, PSP
		STR R4, [R0]
		SUB R0, R0, #4
		STR R5, [R0]
		SUB R0, R0, #4
		STR R6, [R0]
		SUB R0, R0, #4
		STR R7, [R0]
		SUB R0, R0, #4
		STR R8, [R0]
		SUB R0, R0, #4
		STR R9, [R0]
		SUB R0, R0, #4
		STR R10, [R0]
		SUB R0, R0, #4
		STR R11, [R0]
		SUB R0, R0, #4
		BX LR

popR4toR11fromPSP:
		ADD R0, R0, #4
		LDR R11, [R0]
		ADD R0, R0, #4
		LDR R10, [R0]
		ADD R0, R0, #4
		LDR R9, [R0]
		ADD R0, R0, #4
		LDR R8, [R0]
		ADD R0, R0, #4
		LDR R7, [R0]
		ADD R0, R0, #4
		LDR R6, [R0]
		ADD R0, R0, #4
		LDR R5, [R0]
		ADD R0, R0, #4
		LDR R4, [R0]
		BX LR

pushPSP:
		MOV R1, R0
		MRS R0, PSP
		STR R1, [R0]
		SUB R0, R0, #4
		MSR PSP, R0
		ISB
		BX LR

getSVCnumAdd:
		MRS R0, PSP
		ADD R0, R0, #24
		LDR R0, [R0]
		SUB R0, R0, #2
		BX LR

getRfromPSP:
		MOV R1, R0
		MRS R0, PSP
		ADD R0, R0, R1
		LDR R0, [R0]
		BX LR



.endm
