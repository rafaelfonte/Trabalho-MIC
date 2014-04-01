#include <p16f684.inc>

 __CONFIG _FOSC_INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_ON & _FCMEN_ON

    errorlevel  -302                ; suppress message 302 from list file

ADDR_EE_POS EQU 0x00
IrDAb       EQU RA2
IrDAp       EQU PORTA
DEVICE      EQU 0X01    ; MODO TV DO CONTROLE SONY RMT-V297C

STACK_BASE  EQU 0X20
STACK_LIMIT EQU 0X6F


INT_VAR     UDATA_SHR
w_temp      res 1   ; armazena backup do acumulador
w_bk        res 1
status_temp res 1
pos_value   res 1   ; 0 desligado, outros define o n�vel de luminosidade
motor_pos   res 1

IrDA_bits   res 1
IrDA_discard res 1
IrDA_DATA   res 3
IrDA_comand res 1
IrDA_device res 1

record_data res 1
record_addr res 1

RESET_VECTOR    CODE    0x0000     ; processor reset vector
        goto    start              ; go to beginning of program


;============================================================================
; Se��o de tratamento de interrup��es
;
;   ESTA ESTRUTURA FOI UTILIZADA PARA N�O PERMITIR CHAMADAS CONTINUAS DENTRO
;   DESTA FUN��O CASO SEJA CHAMADO MAIS DE UMA INTERRUP��O, SERVE PARA MANTER
;   A HIERARQUIA DE PROCESSAMENTO
;
;============================================================================
INT_VECTOR      CODE    0x0004     ; interrupt vector location

INTERRUPT
        BCF     WDTCON, SWDTEN  ; DESTIVA O WATCH-DOG TIMER
        BCF     INTCON, GIE     ; DESABILITA INTERRUP��O GLOBAL

        CALL    PUSHCTX

        BTFSC   INTCON, INTF    ; TESTA SE A INTERRUP��O OCORREU PELA ENTRADA DO IrDA
        GOTO    IrDA_FALLING
        BTFSC   PIR1, TMR2IF    ; TESTA SE A INTERRUP��O ACONTECEU PELO CONTADOR
        GOTO    IrDA_TIME


END_INTERRUPT
        CALL    POPCTX
        BSF     INTCON, GIE     ; HABILITA INTERRUP��O GLOBAL
        retfie                  ; return from interrupt

IrDA_FALLING
        CALL    IrDA_FALL    ; CHAMA ROTINA DE TRATAMENTO DA RECEP��O DO CONTROLE REMOTO
        GOTO    END_INTERRUPT
IrDA_TIME
        CALL    IrDA_SAMPLER
        GOTO    END_INTERRUPT


;============================================================================
; boostrap da m�quina
;
;
;
;============================================================================

MAIN    CODE

start   ; PR�-SET DO AMBIENTE DE EXECU��O
        CLRF    INTCON

        BSF     STATUS, RP0     ; APONTA PARA BLOCO 1
        MOVLW   b'00000001'
        MOVWF   OPTION_REG      ; DEFINE PRESCALER DO TIMER 0, COLOCA INT/RA2 EM FALLING EDGE
        MOVLW   0X1F            ; b'00011111'
        MOVWF   TRISA           ; DEFINE I/O DA PORTA A
        CLRF    TRISC           ; DEFINE PORTA C COMO SA�DAS
        MOVLW   b'00001000'
        MOVWF   ANSEL           ; DEFINE AS PORTAS QUE S�O ANAL�GICAS
        CLRF    IOCA
        MOVLW   0X07
        MOVWF   WPUA            ; DEFINE AS PORTAS QUE POSSUEM PULL-UP
        ;MOVLW   0X30
        MOVLW   0X10
        MOVWF   ADCON1          ; DEFINE OSCILADOR DEDICADO PARA O CONVERSOR A/D
        MOVLW   b'00000010'
        MOVWF   PIE1            ; HABILITA INTERRUP��O POR TIMER2
        MOVLW   0XE1
        MOVWF   PR2             ; COLOCA 160 NO COMPARADOR DO TIMER 2 (IrDA) 2560 us EM 4MHz


        BCF     STATUS, RP0     ; APONTA PARA O BANCO 0
        CLRF    PORTA           ; INIT PORTA
        CLRF    PORTC
        MOVLW   0X07
        MOVWF   CMCON0          ; DESLIGA OS COMPARADORES
        MOVLW   0X0C
        MOVWF   ADCON0          ; HABILITA ENTRADA ANAL�GICA 3 E AJUSTA PARA SAIDA JUSTIFICADA A ESQUERDA
        MOVLW   0X16
        MOVWF   WDTCON          ; AJUSTA PRESCALER DO WATCH-DOG PARA 1:65536 ~ 2 segundos
        CLRF    PIR1            ; ZERA FLAGS DE INTERRUP��ES DOS PERIF�RICOS


        MOVLW   b'01010000'
        MOVWF   INTCON          ; HABILITA INT/RA2, PERIF�RICOS

        ; Inicaliza��o da RAM
        ;BTFSC   EECON1, WRERR   ; TESTA SE HOUVE ERRO DE GRAVA��O DURANTE RESET
        ;CALL    RECORD_POSITION
        ;CALL    READ_POSITION

        ; Teste da intera��o
        MOVLW   0X70
        MOVWF   pos_value
        ;CALL    RECORD_POSITION

        CALL    INIT_IrDA
        MOVLW   0X84
        MOVWF   IrDA_device

;        CLRF    record_addr
        BSF     INTCON, GIE

        MOVLW   b'00100000'
        XORWF   PORTA, F
        MOVLW   0X20
        MOVWF   FSR
        MOVLW   0x03

LOOP_MAIN   ; loop de execu��o continua

        GOTO    LOOP_MAIN

;============================================================================
;
; Se��o com fun��es
;
;
;============================================================================

FUNCTIONS CODE

;============================================================================
;
;   FUN��ES PUSH-POP PARA IMPLEMENTAR UMA PILHA
;
;============================================================================

PUSHW:      ; COLOCA O CONTE�DO DE W NO TOPO DA PILHA
        MOVWF       INDF          ; COLOCA W NA PILHA
        INCFSZ      FSR, F        ; INCREMENTA PONTEIRO SEM ALTERAR Z
        RETURN

POPW:       ; REMOVE O TOPO DA PILHA E COLOCA EM W
        DECFSZ      FSR, F      ; DECREMENTA PONTEIRO SEM ALTERAR ESTADOS
        SWAPF       INDF, F
        SWAPF       INDF, W     ; COLOCA TOPO DA PILHA EM W
        RETURN

PUSHCTX:    ; SALVA CONTEXTO NA PILHA
        CALL    PUSHW           ; salva w
        SWAPF   STATUS, W
        CALL    PUSHW           ; salva status
        RETURN
POPCTX:     ; RECUPERA CONTEXTO DA PILHA
        CALL    POPW
        MOVWF   status_temp
        SWAPF   status_temp, W   ; restaura status
        MOVWF   STATUS
        CALL    POPW        ; restaura w
        RETURN


;============================================================================
;
;   Delay de 1000 ciclos de clock de instru��o
;
;     Presup�e que a interrup��o de overflow do timer0 est� desativada, caso
;   contr�rio o comportamento � imprevis�vel.
;
;     O timer utiliza um ajuste para se obter o consumo de 1000 ciclos desde
;   a instru��o de chamada at� o retorno na instru��o seguinte
;
;==============================================================================

DELAYms: ; com preescaler em 1:4 desde o call at� o retorno gasta 1000 ciclos

        ; AJUSTA PAR�METROS DO CONTADOR
        ;
        ; PRESCALER 1:4
        ; INCREMENTA NA DESCIDA DO CLOCK
        ; USA O CLOCK DE INSTRU��O COMO BATE DE TEMPO
        ;
        MOVLW   b'11000000'
        ANDWF   OPTION_REG, W
        IORLW   b'00000001'
        MOVWF   OPTION_REG

;la�o do timer
        MOVLW   0X0A                ; AJUSTA CONTADOR PARA GERAR A SA�DA CORRETA
        MOVWF   TMR0                ; ATUALIZA CONTADOR
        BCF     INTCON, T0IF
        BTFSS   INTCON, T0IF
        GOTO $-1
    RETURN


;============================================================================
;
;   INICIALIZA ESTRUTURAS PARA RECEP��O DE UM COMANDO DO CONTROLE REMOTO
;
;============================================================================
INIT_IrDA:
        CLRF    IrDA_bits
        MOVLW   b'00010001'     ; PRESCALER 1:4 POSTSCALER 1:3
        MOVWF   T2CON
        CLRF    TMR2            ; ZERA CONTADOR
        RETURN

;============================================================================
;
; TRATAMENTO DA RECEP��O DOS DADOS DO CONTROLE REMOTO
; PRESUPOE QUE TIMER 2 N�O � USADO POR MAIS NINGUEM
;
;============================================================================
IrDA_FALL:
        BCF     INTCON, INTF    ; ZERA O BIT DA INTERRUP��O
        BTFSC   T2CON, 5        ; TESTA SE � 1:5
        GOTO    IrDA_FALL_SAMPLE_1

        BTFSC   T2CON, TMR2ON
        GOTO    IrDA_FALL_ABORT

        BTFSC   T2CON, 4        ; TESTA SE ESTA NO 1:3
        GOTO    IrDA_FALL_SAMPLE

IrDA_FALL_ABORT
        CALL INIT_IrDA
        RETURN

IrDA_FALL_SAMPLE_1
        BCF     T2CON, 5
IrDA_FALL_SAMPLE
        CLRF    TMR2
        BSF     T2CON, TMR2ON
        RETURN

;===========================================================================
;
;   ROTINA QUE TRATA DA LEITURA DO BIT DO IrDA NA PORTA
;   BYTE 0 - DEVICE
;   BYTE 1 - COMANDO
;   BYTE 2 - EXTEN��O (S� EM 20 BITS)
;
;   INTERRUP��O DEVE CESSAR DEPOIS DE ALTERAR ESTADO DA M�QUINA DE ESTADOS
;===========================================================================

IrDA_SAMPLER:

        BCF     PIR1, TMR2IF    ; ZERA O BIT DA INTERRUP��O
        BTFSC   T2CON, 5        ; TESTA SE � 1:5
        GOTO    IrDA_SAMPLER_5  ; 1:5
        BTFSS   T2CON, 4        ;TESTA DE � 1:3
        GOTO    IrDA_SAMPLER_1
        BTFSS   IrDAp, IrDAb ; TESTA N�VEL L�GICO DA PORTA, DEVE SER ALTO
        GOTO    IrDA_SAMPLER_ABORT
        BCF     T2CON, 4
        GOTO    IrDA_SAMPLER_3

IrDA_SAMPLER_1
        INCF    IrDA_bits, F

        BCF     STATUS, C       ; INVERTE BIT VINDO DA L�GICA
        BTFSS   IrDAp, IrDAb
        BSF     STATUS, C

        ; COLOCA BIT NO ARRAY
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F
        RRF     IrDA_DATA+2, F

IrDA_SAMPLER_3
        BSF     T2CON, 5    ; ATIVA POSTSCALER 1:5
        CLRF    TMR2
        RETURN


; PRESCALER DE 1:5
; ATINGIU O LIMITE DE TEMPO DE ESPERA
; VERIFICA SE A QUANTIDADE DE BITS SE ENQUADRA EM ALGUM PADR�O
IrDA_SAMPLER_5

        MOVLW   0X0C    ; 12 BITS
        XORWF   IrDA_bits, W
        BTFSC   STATUS, Z
        GOTO    IrDA_COMMAND_12_BITS

        MOVLW   0X0F    ; 15 BITS
        XORWF   IrDA_bits, W
        BTFSC   STATUS, Z
        GOTO    IrDA_COMMAND_15_BITS

        MOVLW   0X14    ; 20 BITS
        XORWF   IrDA_bits, W
        BTFSS   STATUS, Z
        CALL    IrDA_SAMPLER_ABORT

; INTERVALO DE C�DIGO QUE CORRIGE POSI��O DOS BITS

        MOVF    IrDA_DATA, W
        MOVWF   w_temp
        MOVF    IrDA_DATA+1, W
        MOVWF   IrDA_DATA
        MOVF    IrDA_DATA+2, W
        MOVWF   IrDA_DATA+1
        MOVF    w_temp, W
        MOVWF   IrDA_DATA+2

IrDA_COMMAND_12_BITS
        MOVLW   b'11110000'
        ANDWF   IrDA_DATA+1, F
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F

IrDA_COMMAND_15_BITS

        BCF     STATUS, C
        RRF     IrDA_DATA+1, F

; FIM DAS CORRE��ES DE POSI��O


        MOVF    IrDA_DATA, W
        XORWF   DEVICE, W
        BTFSS   STATUS, Z           ; testa dispositivo
        GOTO    IrDA_SAMPLER_ABORT


        DECF    IrDA_discard, F
        MOVLW   0x03
        ANDWF   IrDA_discard, F
        BTFSS   STATUS, Z
        GOTO    IrDA_SAMPLER_ABORT  ; descarta envio triplo do controle


        ;MOVF    IrDA_DATA+1, W
        ;XORWF   IrDA_comand, W
        ;BTFSC   STATUS, Z
        ;GOTO    IrDA_SAMPLER_ABORT  ; tratar manter precionada

        ;MOVF    IrDA_DATA+1, W
        ;MOVWF   IrDA_comand

        DECF    IrDA_discard, F

        ; LOCALIZAR A��O NA TABELA
        MOVF    IrDA_DATA+1, W
        CALL    REMOTE_CONTROL_COMMANDS
        IORLW   0X00
        BTFSC   STATUS, Z ; TESTA SE EXISTE A��O PARA ESTA TECLA
        GOTO    IrDA_SAMPLER_ABORT

        MOVWF   PCL
RETORNO_DE_COMANDO
        BCF     STATUS, RP0
        MOVLW   b'00100000'
        XORWF   PORTA, F

IrDA_SAMPLER_ABORT
        CALL    INIT_IrDA       ; INICIALIZA ESTRUTURAS
        RETURN



;=============================================================================
;
;   GERENCIA SEQUENCIA PARA LEITURA E GRAVA��O NA EEPROM
;
;=============================================================================

RECORD_EEPROM:
        BCF     INTCON, GIE     ; DESATIVA INTERRUP��O GLOBAL
        BTFSC   INTCON, GIE     ; TESTA SE INTERRUP��O FOI DESABILITADA
        GOTO    $-2             ; REPETE SE AINDA N�O EST� DESATIVADO
        BSF     STATUS, RP0     ; APONTA PARA BLOCO 1
        BCF     EECON1, WRERR   ; REMOVE MARCA��O DE ERRO DE GRAVA��O
        BSF     EECON1, WREN    ; HABILITA GRAVA��O
        ;MOVF    pos_value, W   ; COLOCA O DADO NO REGISTRADOR DE DADOS
        MOVWF   EEDAT
        MOVF    record_addr, W     ; COLOCA A POSI��O NO REGISTRADOR DE POSI��O
        MOVWF   EEADR
        ; SEQUENCIA DE INICIALIZA��O EXIGIDA PARA A GRAVA��O NA EEPROM
        MOVLW   0X55
        MOVWF   EECON2
        MOVLW   0XAA
        MOVWF   EECON2
        BSF     EECON1, WR      ; ATIVA GRAVA��O
        ; FIM DA SEQUENCIA EXIGIDA
        BTFSC   EECON1, WR      ; TESTA SE GRAVA��O FOI CONCLUIDA
        GOTO $-1
        BCF     STATUS, RP0
        BSF     INTCON, GIE     ; HABILITA INTERRUP��O GLOBAL
        RETURN

READ_EEPROM:
        BCF     INTCON, GIE
        BSF     STATUS, RP0     ; APONTA PARA BLOCO 1
        MOVLW   ADDR_EE_POS
        MOVWF   EEADR
        BSF     EECON1, RD
        BTFSC   EECON1, RD
        GOTO    $-1
        MOVF    EEDAT, W
        MOVWF   pos_value
        BCF     STATUS, RP0
        BSF     INTCON, GIE
        RETURN

;=============================================================================
;
; Rotina que implementa a convers�o A/D do sensor de luminosidade e permanece
; at� que se atinja o n�vel de luminosidade desejado
;
;=============================================================================
LDR_REFRESH:
        BSF     ADCON0, ADON        ; LIGA CONVERSOR A/D
        BSF     ADCON0, GO_NOT_DONE ; INICIA CONVERS�O

AD_DONE
        BTFSC   ADCON0, GO_NOT_DONE ; TESTA SE TERMINOU
        GOTO    AD_DONE
        MOVF    ADRESH, W           ; ARMAZENA O RESULTADO DA CONVERS�O

        SUBWF   pos_value, W        ; COMPARA OS VALORES
        BTFSC   STATUS, Z           ; SE S�O IGUAIS N�O FAZ NADA
        GOTO    REFRESH_END
        BSF     ADCON0, GO_NOT_DONE ; INICIA NOVA CONVERS�O
        BTFSC   STATUS, C
        CALL    MOTOR_DIRECT            ; sentido hor�rio
        BTFSS   STATUS, C
        CALL    MOTOR_REVERSE            ; sentido anti-hor�rio
        CALL    DELAYms
        GOTO    AD_DONE

REFRESH_END
        BCF     ADCON0, ADON    ; DESLIGA CONVERSOR
        CALL    MOTOR_OFF       ; REMOVE CONSUMO DE CORRENTE
        RETURN

;=============================================================================
;
;   GERENCIA O MOTOR DE PASSO CONECTADO AS SA�DAS RC2~RC5

MOTOR_OFF:
        CLRF    w_temp
        GOTO    MOTOR_DO
MOTOR_DIRECT:
        INCF    motor_pos, F
        GOTO    $+2
MOTOR_REVERSE:
        DECF    motor_pos, F

        MOVF    motor_pos, W
        CALL    BIT_PATTERN
        MOVWF   w_temp

MOTOR_DO
        MOVLW   b'11000011'
        ANDWF   PORTC, W
        IORWF   w_temp, W
        MOVWF   PORTC
        RETURN



;=============================================================================
;
;   RETORNA A POSI��O ATUAL DO MOTOR NA TABELA
;
;   N�O � UTIL NO MOMENTO POIS A POSI��O DE PARADA � COM SA�DA EM ZERO
;=============================================================================
MOTOR_STATE:
    BTFSS   PORTC, RC5
    GOTO    MOTOR_STATE_ARM1
    BTFSC   PORTC, RC2
    RETLW   0X07
    BTFSS   PORTC, RC4
    RETLW   0X06
    RETLW   0X05
MOTOR_STATE_ARM1
    BTFSS   PORTC, RC3
    GOTO    MOTOR_STATE_ARM2
    BTFSC   PORTC, RC2
    RETLW   0X01
    BTFSS   PORTC, RC4
    RETLW   0X02
    RETLW   0X03
MOTOR_STATE_ARM2
    BTFSS   PORTC, RC4
    RETLW   0X00
    RETLW   0X04

;=============================================================================
;
;   padr�o de ativa��o das bobinas do motor de passo para meio passo
;
;=============================================================================

BIT_PATTERN:
    ANDLW   0X07        ; remove poss�veis saltos para fora da tabela
    ADDWF   PCL, F      ; move o PC para a posi��o desejada
    RETLW b'00000100'   ; A
    RETLW b'00001100'   ; A~B
    RETLW b'00001000'   ; ~B
    RETLW b'00011000'   ; ~A~B
    RETLW b'00010000'   ; ~A
    RETLW b'00110000'   ; ~AB
    RETLW b'00100000'   ; B
    RETLW b'00100100'   ; AB

;============================================================================
;
;   CONTROLE REMOTO SONY RMT-V297C
;
;   C�DIGO DE DISPOSITIVO
;   TV
;   SIRC12  00001
;
;   VCR
;   SIRC12  01011
;   SIRC15  10111010
;
;
;   KEYMAP:
;
;   1				0x00 	VCR/TV
;   2				0x01 	VCR/TV
;   3				0x02 	VCR/TV
;   4				0x03 	VCR/TV
;   5				0x04 	VCR/TV
;   6				0x05 	VCR/TV
;   7				0x06 	VCR/TV
;   8				0x07 	VCR/TV
;   9				0x08 	VCR/TV
;   0				0x09 	VCR/TV
;   ENTER			0x0B 	VCR/TV
;
;   CH+				0x10 	VCR/TV
;   CH-				0x11	VCR/TV
;   VOL+			0x12	TV
;   VOL-			0x13 	TV
;   X2				0x14 	VCR
;   POWER			0x15 	VCR/TV
;   EJECT			0x16 	VCR
;   AUDIO MONITOR	0x17 	VCR/TV
;   REC				0x1D 	VCR
;   SLOW			0x23 	VCR
;   TV/VIDEO		0x25	TV
;   TV/VIDEO		0x2A 	VCR
;   DISPLAY			0x3A	TV
;   MENU			0x4D 	VCR
;   INPUT SELECT	0x4F 	VCR
;   INDEX RIGHT		0x56 	VCR
;   INDEX LEFT		0x57 	VCR
;   SP/EP			0x58 	VCR
;   DISPLAY			0x5A	VCR
;   EASY TIMER		0x60 	VCR
;   CLEAR			0x63 	VCR
;
;   UP				0x0E 	VCR SIRC15
;   DOWN			0x0F	VCR SIRC15
;   LEFT			0x10 	VCR SIRC15
;   RIGHT			0x11 	VCR SIRC15
;   PLAY/OK			0x18 	VCR SIRC15
;
;
;============================================================================

REMOTE_CONTROL_COMMANDS:
    ANDLW   0X3F        ; remove poss�veis saltos para fora da tabela
    ADDWF   PCL, F      ; move o PC para a posi��o desejada
    RETLW   0X00        ;   1				0x00 	VCR/TV
    RETLW   0X00        ;   2				0x01 	VCR/TV
    RETLW   0X00        ;   3				0x02 	VCR/TV
    RETLW   0X00        ;   4				0x03 	VCR/TV
    RETLW   0X00        ;   5				0x04 	VCR/TV
    RETLW   0X00        ;   6				0x05 	VCR/TV
    RETLW   0X00        ;   7				0x06 	VCR/TV
    RETLW   0X00        ;   8				0x07 	VCR/TV
    RETLW   0X00        ;   9				0x08 	VCR/TV
    RETLW   0X00        ;   0				0x09 	VCR/TV
    RETLW   0X00        ;                   0x0A
    RETLW   0X00        ;   ENTER			0x0B 	VCR/TV
    RETLW   0X00        ;                   0x0C
    RETLW   0X00        ;                   0x0D
    RETLW   0X00        ;                   0x0E
    RETLW   0X00        ;                   0x0F
    RETLW   0X00        ;   CH+				0x10 	VCR/TV
    RETLW   0X00        ;   CH-				0x11	VCR/TV
    RETLW   0X00        ;   VOL+			0x12	TV
    RETLW   0X00        ;   VOL-			0x13 	TV
    RETLW   0X00        ;                   0x14
    RETLW   0X00        ;   POWER			0x15 	VCR/TV
    RETLW   0X00        ;                   0x16
    RETLW   0X00        ;   AUDIO MONITOR	0x17 	VCR/TV
    RETLW   0X00        ;                   0x18
    RETLW   0X00        ;                   0x19
    RETLW   0X00        ;                   0x1A
    RETLW   0X00        ;                   0x1B
    RETLW   0X00        ;                   0x1C
    RETLW   0X00        ;                   0x1D
    RETLW   0X00        ;                   0x1E
    RETLW   0X00        ;                   0x1F
    RETLW   0X00        ;                   0x20
    RETLW   0X00        ;                   0x21
    RETLW   0X00        ;                   0x22
    RETLW   0X00        ;                   0x23
    RETLW   0X00        ;                   0x24
    RETLW   0X00        ;   TV/VIDEO		0x25	TV
    RETLW   0X00        ;                   0x26
    RETLW   0X00        ;                   0x27
    RETLW   0X00        ;                   0x28
    RETLW   0X00        ;                   0x29
    RETLW   0X00        ;                   0x2A
    RETLW   0X00        ;                   0x2B
    RETLW   0X00        ;                   0x2C
    RETLW   0X00        ;                   0x2D
    RETLW   0X00        ;                   0x2E
    RETLW   0X00        ;                   0x2F
    RETLW   0X00        ;                   0x30
    RETLW   0X00        ;                   0x31
    RETLW   0X00        ;                   0x32
    RETLW   0X00        ;                   0x33
    RETLW   0X00        ;                   0x34
    RETLW   0X00        ;                   0x35
    RETLW   0X00        ;                   0x36
    RETLW   0X00        ;                   0x37
    RETLW   0X00        ;                   0x38
    RETLW   0X00        ;                   0x39
    RETLW   0X00        ;   DISPLAY			0x3A	TV
    RETLW   0X00        ;                   0x3B
    RETLW   0X00        ;                   0x3C
    RETLW   0X00        ;                   0x3D
    RETLW   0X00        ;                   0x3E
    RETLW   0X00        ;                   0x3F



    END                         ; directive 'end of program'