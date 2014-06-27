; 	Firmware de controle de basculante automática/manual por sensor de 
;	luminosidade.
;
;	Desenvolvedores:
;		Gilson da Rosa Webber
;		Rafael da Fonte Lopes da Silva 
;
;	Data: Jun 2014
;	
;	Universidade Federal do Rio Grande do Sul - UFRGS
;	Instituto de Informática - INF
;	Depertamento de Engenharia Elétrica - DELET
;
;	ENG04056 - Microcontroladores
;	Prof.: Gilson Inácio Wirth
;
;
#include <p16f684.inc>

 __CONFIG _FOSC_INTOSCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_ON & _FCMEN_ON

    errorlevel  -302                ; suppress message 302 from list file

ADDR_EE_POS     EQU 0x00
IrDAb           EQU RA2     ; bit da porta de recepção do controle
IrDAp           EQU PORTA   ; porta de recepção do controle
ENA_DISP        EQU RA5     ; bit da porta de controle do display
DISPLAYp        EQU PORTA   ; porta de controle do display
DEVICE          EQU 0X01    ; MODO TV DO CONTROLE SONY RMT-V297C
ST_AUTO         EQU 0       ; Estado de operação automática
ST_COMMAND      EQU 1       ; Estado de execução de um comando
ST_DISP_SET     EQU 2
ST_WRITE_CHANGE EQU 3
ST_MOVE_ALL     EQU 4




                UDATA   0X50; Inicio da área de variáveis (RAM) - Bloco 0
IrDA_bits       res 1       ; quantidade de bits enviados pelo controle
IrDA_discard    res 1       ; contador para discarte de envio multiplo do controle
IrDA_DATA       res 3       ; dados recebidos do controle remoto
IrDA_command    res 1       ; commando enviado pelo controle remoto
SMCON           res 1       ; controle da máquina de estados
count_disp      res 1       ; contador de bits enviados para o display
w_disp          res 1       ; temporário para operações de envio serial

pos_value       res 1       ; 0 desligado, outros define o nível de luminosidade
motor_pos       res 1       ; quais bobinas foram acionadas na ultima chamada
counter         res 1       ; contador multiuso

B1              res 1       ; digito um do display 7-seg
B2              res 1       ; digito dois do display 7-seg
B1_temp         res 1       ; temporário
B2_temp         res 1       ; temporário
LOOP_BCD_COUNTER    res 1   ; ?
VALUE_BIN       res 1       ; valor binário do display, não usado no momento
com_routine     res 1       ; armazena a função a ser executado pelo comando


                UDATA_SHR   ; área de memória compartilhada entre blocos
w_temp          res 1       ; acumulador temporário
w_bk            res 1       ; armazena backup do acumulador
status_bk       res 1       ; backup do registrador de status


record_data     res 1       ; dado a ser gravado na eeprom
record_addr     res 1       ; endereço a ser gravado na eeprom


RESET_VECTOR    CODE    0x0000  ; ponto de execução em caso de reset
        goto    start           ; salta para o inicio da execução


;============================================================================
; Seção de tratamento de interrupções
;
;   ESTA ESTRUTURA FOI UTILIZADA PARA NÃO PERMITIR CHAMADAS CONTINUAS DENTRO
;   DESTA FUNÇÃO CASO SEJA CHAMADO MAIS DE UMA INTERRUPÇÃO, SERVE PARA MANTER
;   A HIERARQUIA DE PROCESSAMENTO
;
;============================================================================
INT_VECTOR      CODE    0x0004  ; interrupt vector location

INTERRUPT
        BCF     INTCON, GIE     ; DESABILITA INTERRUPÇÃO GLOBAL
        BCF     WDTCON, SWDTEN  ; DESTIVA O WATCH-DOG

        movwf   w_bk            ; salva o conteudo do acumuludaor
        movf    STATUS, W       ; copia o conteudo do registrador de status
        movwf   status_bk       ; salva o conteúdo do registrador de status

        BTFSC   INTCON, INTF    ; Interrupção pelo controle remoto
        GOTO    IrDA_FALLING
        BTFSC   PIR1, TMR2IF    ; Interrupção pelo contador (controle remoto)
        GOTO    IrDA_TIME

;RESOLVER                       ; interrupção não tratável, fica em loop
END_INTERRUPT
        movf    status_bk, W    ; retrieve copy of STATUS register
        movwf   STATUS          ; restore pre-isr STATUS register contents
        swapf   w_bk, F
        swapf   w_bk, W         ; restore pre-isr W register contents

        BSF     INTCON, GIE     ; HABILITA INTERRUPÇÃO GLOBAL
        retfie                  ; return from interrupt

IrDA_FALLING
        BCF     INTCON, INTF    ; zera bit da interrupção por controle remoto
        CALL    IrDA_FALL       ; tratamento da recepção
        GOTO    END_INTERRUPT
IrDA_TIME
        BCF     PIR1, TMR2IF    ; zera bit de interrupção pelo contador
        CALL    IrDA_SAMPLER    ; executa leitura do bit recebido
        GOTO    END_INTERRUPT




;============================================================================
; boostrap da máquina
;
;
;
;============================================================================

MAIN    CODE

start   ; PRÉ-SET DO AMBIENTE DE EXECUÇÃO
        CLRF    INTCON

        BSF     STATUS, RP0     ; APONTA PARA BLOCO 1
        MOVLW   b'00000001'
        MOVWF   OPTION_REG      ; DEFINE PRESCALER DO TIMER 0, COLOCA INT/RA2 EM FALLING EDGE
        MOVLW   0X1F            ; b'00011111'
        MOVWF   TRISA           ; DEFINE I/O DA PORTA A
        CLRF    TRISC           ; DEFINE PORTA C COMO SAÍDAS
        MOVLW   b'00001000'
        MOVWF   ANSEL           ; DEFINE AS PORTAS QUE SÃO ANALÓGICAS
        CLRF    IOCA            ; Desativa InterrupOnChange para todas as portas
        MOVLW   b'00000111'
        MOVWF   WPUA            ; Ajusta PULL-UP
        MOVLW   0X10
        MOVWF   ADCON1          ; DEFINE OSCILADOR 1/8 ~ 2us PARA O CONVERSOR A/D
        MOVLW   b'00000010'
        MOVWF   PIE1            ; HABILITA INTERRUPÇÃO POR TIMER2
        MOVLW   0XE1
        MOVWF   PR2             ; COLOCA 160 NO COMPARADOR DO TIMER 2 (IrDA) 2560 us EM 4MHz


        BCF     STATUS, RP0     ; APONTA PARA O BANCO 0
        CLRF    PORTA           ; INIT PORTA
        CLRF    PORTC
        MOVLW   0X07
        MOVWF   CMCON0          ; DESLIGA OS COMPARADORES
        MOVLW   0X0C
        MOVWF   ADCON0          ; HABILITA ENTRADA ANALÓGICA 3 E AJUSTA PARA SAIDA JUSTIFICADA A ESQUERDA
        MOVLW   0X16
        MOVWF   WDTCON          ; AJUSTA PRESCALER DO WATCH-DOG PARA 1:65536 ~ 2 segundos
        CLRF    PIR1            ; ZERA FLAGS DE INTERRUPÇÕES DOS PERIFÉRICOS

        MOVLW   b'01010000'
        MOVWF   INTCON          ; HABILITA INT/RA2, PERIFÉRICOS

        ; Inicialização de variáveis

        ; Inicalização da RAM
        ;BTFSC   EECON1, WRERR   ; TESTA SE HOUVE ERRO DE GRAVAÇÃO DURANTE RESET
        ;CALL    RECORD_POSITION
        ;CALL    READ_POSITION

        ; Teste da interação
        MOVLW   0X80            ; 50%
        MOVWF   pos_value
        ;CALL    RECORD_POSITION

;        CLRF    record_addr

        CLRF    SMCON           ; inicializa estrutura da maquina de estados
        MOVLW   0xFF
        MOVWF   IrDA_command    ; inicializa comando do controle remoto
        CALL    INIT_IrDA       ; inicializa estruturas de recepção do controle remoto
        CLRF    B1              ; inicializa byte 1 do display 7-seg
        CLRF    B2              ; inicializa byte 2 do 7-seg

        BSF     DISPLAYp, ENA_DISP

        BSF     INTCON, GIE     ; Habilita interrupções

MAIN_1
        BTFSC   SMCON, ST_COMMAND
        call    COMMANDS
        BTFSS   SMCON, ST_DISP_SET
        call    READ_LDR
        BTFSC   SMCON, ST_AUTO
        call    AUTO_REFRESH
        goto    MAIN_1

        GOTO    start           ; ocorreu algum erro, resetar


;============================================================================
;
; Seção com funções
;
;
;============================================================================

FUNCTIONS CODE

;============================================================================
;
;   Delay de 1000 ciclos de clock de instrução
;
;     Presupõe que a interrupção de overflow do timer0 está desativada, caso
;   contrário o comportamento é imprevisível.
;
;     O timer utiliza um ajuste para se obter o consumo de 1000 ciclos desde
;   a instrução de chamada até o retorno na instrução seguinte
;
;==============================================================================

DELAYms: ; com preescaler em 1:4 desde o call até o retorno gasta 1000 ciclos

        ; AJUSTA PARÂMETROS DO CONTADOR
        ;
        ; PRESCALER 1:4
        ; INCREMENTA NA DESCIDA DO CLOCK
        ; USA O CLOCK DE INSTRUÇÃO COMO BATE DE TEMPO
        ;
        MOVLW   b'11000000'
        ANDWF   OPTION_REG, W
        IORLW   b'00000001'
        MOVWF   OPTION_REG

;laço do timer
        MOVLW   0X0A                ; AJUSTA CONTADOR PARA GERAR A SAÍDA CORRETA
        MOVWF   TMR0                ; ATUALIZA CONTADOR
        BCF     INTCON, T0IF
        BTFSS   INTCON, T0IF
        GOTO $-1
    RETURN


;============================================================================
;
;   INICIALIZA ESTRUTURAS PARA RECEPÇÃO DE UM COMANDO DO CONTROLE REMOTO
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
; TRATAMENTO DA RECEPÇÃO DOS DADOS DO CONTROLE REMOTO
; PRESUPOE QUE TIMER 2 NÃO É USADO POR MAIS NINGUEM
;
;============================================================================
IrDA_FALL:
        BTFSC   T2CON, 6    ; testa se está em aguardo de novo comando 1:16
        CALL    INIT_IrDA

        BTFSC   T2CON, 5
        GOTO    IrDA_FALL_SAMPLE_1  ; se 1:5

        BTFSC   T2CON, TMR2ON
        GOTO    IrDA_FALL_ABORT     ; se não é 1:5 e está com o timer ligado

        BTFSC   T2CON, 4
        GOTO    IrDA_FALL_SAMPLE    ; se 1:3

IrDA_FALL_ABORT
        CALL    INIT_IrDA
        RETURN

IrDA_FALL_SAMPLE_1
        BCF     T2CON, 5        ; passa para 1:1
IrDA_FALL_SAMPLE
        CLRF    TMR2
        BSF     T2CON, TMR2ON
        RETURN

;===========================================================================
;
;   ROTINA QUE TRATA DA LEITURA DO BIT DO IrDA NA PORTA
;   BYTE 0 - DEVICE
;   BYTE 1 - COMANDO
;   BYTE 2 - EXTENÇÃO (SÓ EM 20 BITS)
;
;   INTERRUPÇÃO DEVE CESSAR DEPOIS DE ALTERAR ESTADO DA MÁQUINA DE ESTADOS
;===========================================================================

IrDA_SAMPLER:
        BTFSC   T2CON, 6
        GOTO    IrDA_SAMPLER_RESET      ; tem que executar INIT e zerar contador
        BTFSC   T2CON, 5        ; TESTA SE É 1:5
        GOTO    IrDA_SAMPLER_5  ; 1:5
        BTFSS   T2CON, 4        ;TESTA DE É 1:3
        GOTO    IrDA_SAMPLER_1
        BTFSC   IrDAp, IrDAb    ; TESTA NÍVEL LÓGICO DA PORTA, DEVE SER ALTO
        GOTO    IrDA_SAMPLER_3
        goto    IrDA_SAMPLER_RESET
        RETURN

IrDA_SAMPLER_1
        INCF    IrDA_bits, F

        BCF     STATUS, C       ; INVERTE BIT VINDO DA LÓGICA
        BTFSS   IrDAp, IrDAb
        BSF     STATUS, C

        ; COLOCA BIT NO ARRAY
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F
        RRF     IrDA_DATA+2, F

IrDA_SAMPLER_3
        BCF     T2CON, 4        ; passa para 1:1
        BSF     T2CON, 5        ; ATIVA POSTSCALER 1:5
        CLRF    TMR2            ; mantem contador ativo
        RETURN


; PRESCALER DE 1:5
; ATINGIU O LIMITE DE TEMPO DE ESPERA
; VERIFICA SE A QUANTIDADE DE BITS SE ENQUADRA EM ALGUM PADRÃO
IrDA_SAMPLER_5
        MOVLW   b'01111111'
        MOVWF   T2CON        ; ativa pos e pre 1:16

        MOVLW   0X0C            ; 12 BITS
        XORWF   IrDA_bits, W
        BTFSC   STATUS, Z
        GOTO    IrDA_COMMAND_12_BITS

        MOVLW   0X0F            ; 15 BITS
        XORWF   IrDA_bits, W
        BTFSC   STATUS, Z
        GOTO    IrDA_COMMAND_15_BITS

        MOVLW   0X14            ; 20 BITS
        XORWF   IrDA_bits, W
        BTFSS   STATUS, Z
        goto    IrDA_SAMPLER_RESET

; INTERVALO DE CÓDIGO QUE CORRIGE POSIÇÃO DOS BITS

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
        BCF     STATUS, C
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F
        RRF     IrDA_DATA, F
        RRF     IrDA_DATA+1, F

IrDA_COMMAND_15_BITS

        BCF     STATUS, C
        RRF     IrDA_DATA+1, F

; FIM DAS CORREÇÕES DE POSIÇÃO


        MOVF    IrDA_DATA, W
        XORLW   DEVICE
        BTFSS   STATUS, Z           ; testa dispositivo
        RETURN

        ; TODO: tratar melhor o descarte do envio triplo
        ; comando diferente do anterior, zera contador
        ; commando igual incrementa e testa
        ; caso atinja 2, zera comando
        MOVF    IrDA_command, W
        XORWF   IrDA_DATA+1, W
        BTFSS   STATUS, Z           ; testa se comando é igual ao anterior
        CLRF    IrDA_discard
        INCF    IrDA_discard, F
        MOVLW   0x03
        XORWF   IrDA_discard, W
        BTFSC   STATUS, Z
        CLRF    IrDA_discard
        MOVLW   0x01
        XORWF   IrDA_discard, W
        BTFSS   STATUS, Z
        RETURN

        MOVF    IrDA_DATA+1, W
        MOVWF   IrDA_command

        ; LOCALIZAR AÇÃO NA TABELA
        ;MOVF    IrDA_DATA+1, W
        BTFSC   SMCON, ST_AUTO     ; testa qual tabela deve ser consultada auto/manual
        CALL    REMOTE_CONTROL_COMMANDS_AUTO
        BTFSS   SMCON, ST_AUTO
        CALL    REMOTE_CONTROL_COMMANDS_MANUAL

        MOVWF   com_routine

        ANDLW   0xFF
        BTFSS   STATUS, Z
        ;CALL    COMMANDS
        BSF     SMCON, ST_COMMAND
        RETURN

IrDA_SAMPLER_RESET
        ;MOVLW   0xFF
        ;MOVWF   IrDA_command
        CLRF    IrDA_discard
        CALL    INIT_IrDA       ; INICIALIZA ESTRUTURAS
        RETURN



;=============================================================================
;
;   GERENCIA SEQUENCIA PARA LEITURA E GRAVAÇÃO NA EEPROM
;
;=============================================================================

RECORD_EEPROM:
        BCF     INTCON, GIE     ; DESATIVA INTERRUPÇÃO GLOBAL
        BTFSC   INTCON, GIE     ; TESTA SE INTERRUPÇÃO FOI DESABILITADA
        GOTO    $-2             ; REPETE SE AINDA NÃO ESTÁ DESATIVADO
        BSF     STATUS, RP0     ; APONTA PARA BLOCO 1
        BCF     EECON1, WRERR   ; REMOVE MARCAÇÃO DE ERRO DE GRAVAÇÃO
        BSF     EECON1, WREN    ; HABILITA GRAVAÇÃO
        ;MOVF    pos_value, W   ; COLOCA O DADO NO REGISTRADOR DE DADOS
        MOVWF   EEDAT
        MOVF    record_addr, W     ; COLOCA A POSIÇÃO NO REGISTRADOR DE POSIÇÃO
        MOVWF   EEADR
        ; SEQUENCIA DE INICIALIZAÇÃO EXIGIDA PARA A GRAVAÇÃO NA EEPROM
        MOVLW   0X55
        MOVWF   EECON2
        MOVLW   0XAA
        MOVWF   EECON2
        BSF     EECON1, WR      ; ATIVA GRAVAÇÃO
        ; FIM DA SEQUENCIA EXIGIDA
        BTFSC   EECON1, WR      ; TESTA SE GRAVAÇÃO FOI CONCLUIDA
        GOTO $-1
        BCF     STATUS, RP0
        BSF     INTCON, GIE     ; HABILITA INTERRUPÇÃO GLOBAL
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
; Rotina que implementa a conversão A/D do sensor de luminosidade e permanece
; até que se atinja o nível de luminosidade desejado
;
;=============================================================================
AUTO_REFRESH:
        BSF     ADCON0, ADON        ; LIGA CONVERSOR A/D
        BSF     ADCON0, GO_NOT_DONE ; INICIA CONVERSÃO

AD_DONE
        BTFSC   ADCON0, GO_NOT_DONE ; TESTA SE TERMINOU
        GOTO    AD_DONE
        MOVF    ADRESH, W           ; ARMAZENA O RESULTADO DA CONVERSÃO



        SUBWF   pos_value, W        ; COMPARA OS VALORES
        BTFSC   STATUS, Z           ; SE SÃO IGUAIS NÃO FAZ NADA
        GOTO    REFRESH_watch

        BTFSS   STATUS, C
        goto    AD_FORWARD

        BTFSS   PORTA, RA1
        goto    REFRESH_END
        CALL    MOTOR_REVERSE       ; fechar
        goto    REFRESH_END

AD_FORWARD
        BTFSS   PORTA, RA0
        goto    REFRESH_END
        CALL    MOTOR_DIRECT        ; abrir

REFRESH_END
        BCF     ADCON0, ADON    ; DESLIGA CONVERSOR
        ;CALL    MOTOR_OFF       ; REMOVE CONSUMO DE CORRENTE
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        CALL    DELAYms
        RETURN

REFRESH_watch
        BCF     ADCON0, ADON    ; DESLIGA CONVERSOR

        BSF     WDTCON, SWDTEN
        SLEEP
        BCF     WDTCON, SWDTEN

        return

;=============================================================================
;
;   GERENCIA O MOTOR DE PASSO CONECTADO AS SAÍDAS RC2~RC5

MOTOR_OFF:
        CLRF    w_temp
        GOTO    MOTOR_DO
MOTOR_DIRECT:
        BTFSS   PORTA, RA0
        RETURN
        INCF    motor_pos, F
        GOTO    $+4
MOTOR_REVERSE:
        BTFSS   PORTA, RA1
        RETURN
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

;-----------------------------------------------------------------------------
;
;   Conversor binário -> BCD
;
;-----------------------------------------------------------------------------

BCD_CONVERT:
    MOVWF VALUE_BIN
	;inicializa laco (8 vezes)
	MOVLW 0x80
	MOVWF LOOP_BCD_COUNTER
    CLRF B1                     ;b1; - primeiro byte com dois nibbles de bcd
    CLRF B2                     ;b2; - segundo byte, com um unico nibble de bcd

	GOTO KEEP_SHIFTING
LOOP_SHIFT
	BTFSC B1, 3                 ;Testa se primeiro nibble eh maior ou igual a oito
	GOTO ADD_B1_0
	BTFSS B1, 2                 ;Testa se eh maior ou igual a quatro
	GOTO TEST_B1_1				;nibble ok, continuar
	BTFSC B1, 1
	GOTO ADD_B1_0				;tem que somar
	BTFSS B1, 0
	GOTO TEST_B1_1

ADD_B1_0						;primeiro nibble eh maior que 5
	MOVLW 0x03
	ADDWF B1, f					;adiciona 3 ao nibble

TEST_B1_1
	BTFSC B1, 7                 ;Testa se primeiro nibble eh maior ou igual a oito
	GOTO ADD_B1_1
	BTFSS B1, 6                 ;Testa se eh maior ou igual a quatro
	GOTO TEST_B2				;nibble ok, continuar
	BTFSC B1, 5
	GOTO ADD_B1_1				;tem que somar
	BTFSS B1, 4
	GOTO TEST_B2

ADD_B1_1						;segundo nibble eh maior que 5
	MOVLW 0x30
	ADDWF B1, f					;adiciona 3 ao nibble

TEST_B2
	BTFSC B2, 3                 ;Testa se primeiro nibble eh maior ou igual a oito
	GOTO ADD_B2
	BTFSS B2, 2                 ;Testa se eh maior ou igual a quatro
	GOTO KEEP_SHIFTING			;nibble ok, continuar
	BTFSC B2, 1
	GOTO ADD_B2					;tem que somar
	BTFSS B2, 0
	GOTO KEEP_SHIFTING

ADD_B2
	MOVLW 0x03
	ADDWF B2, f                 ;adiciona 3 ao nibble

KEEP_SHIFTING
	RLF VALUE_BIN
	RLF B1                      ;joga bit de carry no primeiro bit do registrador, shifta o registrador
	RLF B2                      ;de novo, passa ultimo bit adiante

	RRF LOOP_BCD_COUNTER,f
	MOVF LOOP_BCD_COUNTER,f		;testa se valor do registrador eh zero
	BTFSS STATUS,Z              ;testa se chegamos ao fim da execucao
	GOTO LOOP_SHIFT
final_da_execucao               ;realizar tratamentos finais
	RETURN

;=============================================================================
;
; Envia os valores bcd em B1 e B2 para o display
;
; a sensibilidade do clock do receptor do display é a borda de subida
; para habilitar a saída manter enable em 0 para atualizar o display enviar pulso
; de clock na linha de enable.
;
; o shift é de 16 bits
;
;=============================================================================
SEND_DISPLAY:
    BSF     SMCON, 3
    SWAPF   B1, F

START_SEND
    MOVF    B1, W
    CALL    SEVEN_SEG_CODE_LOOKUP
    MOVWF   w_disp
    MOVLW   0X08
    MOVWF   count_disp
NEXT_BIT
    RRF     w_disp, F

    ; DEFINE BIT DE DADOS
    BTFSS   STATUS, C
    BCF     PORTC, 0
    BTFSC   STATUS, C
    BSF     PORTC, 0

    ; APLICA TRANSIÇÃO
    BCF     PORTC, 1
    BSF     PORTC, 1
    BCF     PORTC, 1

    DECF    count_disp, F
    BTFSS   STATUS, Z
    GOTO    NEXT_BIT

    SWAPF   B1, F
    BTFSS   SMCON, 3
    GOTO    END_SEND
    BCF     SMCON, 3
    GOTO    START_SEND


END_SEND
    ; envia pulso de clock para o display atualizar
    SWAPF   B1, F
    BCF     DISPLAYp, ENA_DISP
    BSF     DISPLAYp, ENA_DISP
    BCF     DISPLAYp, ENA_DISP
    RETURN


;============================================================================
;
;	Envia para o display a leitura instantânea do sensor de luminosidade
;
;============================================================================
READ_LDR:
        BTFSC   DISPLAYp, ENA_DISP
        return
        BSF     ADCON0, ADON        ; LIGA CONVERSOR A/D
        CALL    DELAYms
        BSF     ADCON0, GO_NOT_DONE ; INICIA CONVERSÃO
        BTFSC   ADCON0, GO_NOT_DONE ; TESTA SE TERMINOU
        GOTO    $-1
        BCF     ADCON0, ADON        ; desliga CONVERSOR A/D
        MOVF    ADRESH, W           ; ARMAZENA O RESULTADO DA CONVERSÃO
        MOVWF   B1
        CLRF    B2
        CALL    SEND_DISPLAY
        RETURN


;=============================================================================
;
;   padrão de ativação das bobinas do motor de passo para meio passo
;
;=============================================================================
    org     0x0200  ; AJUSTA INICIO DAS TABELAS, 255 POSIÇÕES
BIT_PATTERN:
    CLRF    PCLATH
    BSF     PCLATH, 1   ; ACERTA DESLOCAMENTO SOMADO AO PCL
    ANDLW   0X07        ; remove possíveis saltos para fora da tabela
    ADDWF   PCL, F      ; move o PC para a posição desejada
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
;   CÓDIGO DE DISPOSITIVO
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

REMOTE_CONTROL_COMMANDS_AUTO:
    CLRF    PCLATH
    BSF     PCLATH, 1   ; ACERTA O DESLOCAMENTO SOMADO AO PCL
    ANDLW   0X3F        ; remove possíveis saltos para fora da tabela
    ADDWF   PCL, F      ; move o PC para a posição desejada
    RETLW   cmd_digit_1   ;   1				0x00 	VCR/TV
    RETLW   cmd_digit_2   ;   2				0x01 	VCR/TV
    RETLW   cmd_digit_3   ;   3				0x02 	VCR/TV
    RETLW   cmd_digit_4   ;   4				0x03 	VCR/TV
    RETLW   cmd_digit_5   ;   5				0x04 	VCR/TV
    RETLW   cmd_digit_6   ;   6				0x05 	VCR/TV
    RETLW   cmd_digit_7   ;   7				0x06 	VCR/TV
    RETLW   cmd_digit_8   ;   8				0x07 	VCR/TV
    RETLW   cmd_digit_9   ;   9				0x08 	VCR/TV
    RETLW   cmd_digit_0   ;   0				0x09 	VCR/TV
    RETLW   0X00        ;                   0x0A
    RETLW   cmd_apply   ;   ENTER			0x0B 	VCR/TV
    RETLW   0X00        ;                   0x0C
    RETLW   0X00        ;                   0x0D
    RETLW   0X00        ;                   0x0E
    RETLW   0X00        ;                   0x0F
    RETLW   cmd_upper_unit        ;   CH+				0x10 	VCR/TV
    RETLW   cmd_lower_unit        ;   CH-				0x11	VCR/TV
    RETLW   cmd_upper   ;   VOL+			0x12	TV
    RETLW   cmd_lower   ;   VOL-			0x13 	TV
    RETLW   0X00        ;                   0x14
    RETLW   cmd_manual  ;   POWER			0x15 	VCR/TV
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
    RETLW   cmd_display ;   DISPLAY			0x3A	TV
    RETLW   0X00        ;                   0x3B
    RETLW   0X00        ;                   0x3C
    RETLW   0X00        ;                   0x3D
    RETLW   0X00        ;                   0x3E
    RETLW   0X00        ;                   0x3F

REMOTE_CONTROL_COMMANDS_MANUAL:
    CLRF    PCLATH
    BSF     PCLATH, 1       ; ACERTA O DESLOCAMENTO SOMADO AO PCL
    ANDLW   0X3F            ; remove possíveis saltos para fora da tabela
    ADDWF   PCL, F          ; move o PC para a posição desejada
    RETLW   0X00            ;   1				0x00 	VCR/TV
    RETLW   0X00            ;   2				0x01 	VCR/TV
    RETLW   0X00            ;   3				0x02 	VCR/TV
    RETLW   0X00            ;   4				0x03 	VCR/TV
    RETLW   0X00            ;   5				0x04 	VCR/TV
    RETLW   0X00            ;   6				0x05 	VCR/TV
    RETLW   0X00            ;   7				0x06 	VCR/TV
    RETLW   0X00            ;   8				0x07 	VCR/TV
    RETLW   0X00            ;   9				0x08 	VCR/TV
    RETLW   0X00            ;   0				0x09 	VCR/TV
    RETLW   0X00            ;                   0x0A
    RETLW   0X00            ;   ENTER			0x0B 	VCR/TV
    RETLW   0X00            ;                   0x0C
    RETLW   0X00            ;                   0x0D
    RETLW   0X00            ;                   0x0E
    RETLW   0X00            ;                   0x0F
    RETLW   cmd_open_all    ;   CH+				0x10 	VCR/TV
    RETLW   cmd_close_all   ;   CH-				0x11	VCR/TV
    RETLW   cmd_open        ;   VOL+			0x12	TV
    RETLW   cmd_close       ;   VOL-			0x13 	TV
    RETLW   0X00            ;                   0x14
    RETLW   cmd_auto        ;   POWER			0x15 	VCR/TV
    RETLW   0X00            ;                   0x16
    RETLW   0X00            ;   AUDIO MONITOR	0x17 	VCR/TV
    RETLW   0X00            ;                   0x18
    RETLW   0X00            ;                   0x19
    RETLW   0X00            ;                   0x1A
    RETLW   0X00            ;                   0x1B
    RETLW   0X00            ;                   0x1C
    RETLW   0X00            ;                   0x1D
    RETLW   0X00            ;                   0x1E
    RETLW   0X00            ;                   0x1F
    RETLW   0X00            ;                   0x20
    RETLW   0X00            ;                   0x21
    RETLW   0X00            ;                   0x22
    RETLW   0X00            ;                   0x23
    RETLW   0X00            ;                   0x24
    RETLW   0X00            ;   TV/VIDEO		0x25	TV
    RETLW   0X00            ;                   0x26
    RETLW   0X00            ;                   0x27
    RETLW   0X00            ;                   0x28
    RETLW   0X00            ;                   0x29
    RETLW   0X00            ;                   0x2A
    RETLW   0X00            ;                   0x2B
    RETLW   0X00            ;                   0x2C
    RETLW   0X00            ;                   0x2D
    RETLW   0X00            ;                   0x2E
    RETLW   0X00            ;                   0x2F
    RETLW   0X00            ;                   0x30
    RETLW   0X00            ;                   0x31
    RETLW   0X00            ;                   0x32
    RETLW   0X00            ;                   0x33
    RETLW   0X00            ;                   0x34
    RETLW   0X00            ;                   0x35
    RETLW   0X00            ;                   0x36
    RETLW   0X00            ;                   0x37
    RETLW   0X00            ;                   0x38
    RETLW   0X00            ;                   0x39
    RETLW   cmd_display     ;   DISPLAY			0x3A	TV
    RETLW   0X00            ;                   0x3B
    RETLW   0X00            ;                   0x3C
    RETLW   0X00            ;                   0x3D
    RETLW   0X00            ;                   0x3E
    RETLW   0X00            ;                   0x3F



;TABELA PARA O DISPLAY DE SETE SEGMENTOS
;
; Lógica negada
;
;       a
;    f     b
;       g
;    e     c
;       d
;
;   byte = (dot)abcdefg

SEVEN_SEG_CODE_LOOKUP:
    CLRF    PCLATH
    BSF PCLATH, 1
    ANDLW   0x0F
    ADDWF PCL, F
    RETLW b'10000001'           ;0
    RETLW b'11001111'           ;1
    RETLW b'10010010'           ;2
    RETLW b'10000110'           ;3
    RETLW b'11001100'           ;4
    RETLW b'10100100'           ;5
    RETLW b'10100000'           ;6
    RETLW b'10001111'           ;7
    RETLW b'10000000'           ;8
    RETLW b'10000100'           ;9
    RETLW b'10001000'           ;a
    RETLW b'11100000'           ;b
    RETLW b'10110001'           ;c
    RETLW b'11000010'           ;d
    RETLW b'10110000'           ;e
    RETLW b'10111000'           ;f

; tabela com valores a serem somados no conversor bcd -> binário
BCD_TO_BINARY_DOZENS:
    MOVWF   w_temp
    SUBLW   0X09
    BTFSC   STATUS, C
    RETLW   0XFF
    BSF     PCLATH, 1
    ADDWF   PCL, F
    RETLW   0x00        ; 0
    RETLW   0x0A        ; 10
    RETLW   0x14        ; 20
    RETLW   0x1E        ; 30
    RETLW   0x28        ; 40
    RETLW   0x32        ; 50
    RETLW   0x3C        ; 60
    RETLW   0x46        ; 70
    RETLW   0x50        ; 80
    RETLW   0x5A        ; 90

;------------------------------------------------------------------------------
;
; Funções que devem ser chamadas alterando o pcl, não devem ser chamadas com
; CALL, ver limite de endereço ao gerar o binário para se necessário ajustar o
; PCLATCH
;
;
;------------------------------------------------------------------------------
    org     0x0300
COMMANDS:
        BCF     SMCON, ST_COMMAND
        MOVLW   0X03
        MOVWF   PCLATH
        MOVF    com_routine, W
        MOVWF   PCL

cmd_upper_unit
        INCF    pos_value, W
        BTFSC   SMCON, ST_WRITE_CHANGE
        INCF    B1_temp, W
        goto    cmd_unit
cmd_lower_unit
        DECF    pos_value, W
        BTFSC   SMCON, ST_WRITE_CHANGE
        DECF    B1_temp, W
cmd_unit
        ANDLW   0x0F
        MOVWF   w_temp

        MOVLW   0xF0

        BTFSS   SMCON, ST_WRITE_CHANGE
        ANDWF   pos_value, W
        BTFSC   SMCON, ST_WRITE_CHANGE
        ANDWF   B1_temp, W

        IORWF   w_temp, W
        MOVWF   B1_temp
        MOVWF   B1
        CALL    SEND_DISPLAY
        BSF     SMCON, ST_WRITE_CHANGE
        RETURN

cmd_digit_0
        MOVLW 0x00
        goto cmd_digit
cmd_digit_1
        MOVLW 0x10
        goto cmd_digit
cmd_digit_2
        MOVLW 0x20
        goto cmd_digit
cmd_digit_3
        MOVLW 0x30
        goto cmd_digit
cmd_digit_4
        MOVLW 0x40
        goto cmd_digit
cmd_digit_5
        MOVLW 0x50
        goto cmd_digit
cmd_digit_6
        MOVLW 0x60
        goto cmd_digit
cmd_digit_7
        MOVLW 0x70
        goto cmd_digit
cmd_digit_8
        MOVLW 0x80
        goto cmd_digit
cmd_digit_9
        MOVLW 0x90

cmd_digit
        MOVWF   w_temp
        MOVF    B1_temp, W

        BTFSS   SMCON, ST_WRITE_CHANGE ; testa se já houve modificação anterior
        MOVF    pos_value, W

CONTINUE_DIGIT
        ANDLW   0x0F
        IORWF   w_temp, F
        SWAPF   w_temp, W
        MOVWF   B1_temp
        MOVWF   B1
        CALL    SEND_DISPLAY
        BSF     SMCON, ST_WRITE_CHANGE ; ativa dado a ser gravado
        RETURN

cmd_apply
        ; tratar conversão bcd
        BTFSS   SMCON, ST_WRITE_CHANGE ; ativo se há mudança a ser salva
        RETURN
        MOVF    B1_temp, W
        MOVWF   pos_value
        goto    cmd_change_record
cmd_upper
        INCF    pos_value, W
        BTFSS   STATUS, Z
        MOVWF   pos_value
        goto    cmd_change_record
cmd_lower
        MOVF    pos_value, F
        BTFSS   STATUS, Z
        DECF    pos_value, F
cmd_change_record
        ; gravar na eeprom
        BCF     SMCON, ST_WRITE_CHANGE    ; informa que não há nada a ser gravado
        CLRF    B1_temp      ; descarta temporarios
        CLRF    B2_temp
        goto    cmd_display_show

cmd_display
        BTFSS   DISPLAYp, ENA_DISP
        goto    $+3
        BCF     DISPLAYp, ENA_DISP
        return

        MOVLW   b'00000100'
        XORWF   SMCON, F

        BTFSS   SMCON, ST_DISP_SET
        BSF     DISPLAYp, ENA_DISP

        BTFSC   DISPLAYp, ENA_DISP
        RETURN  ; se ação foi desligar o display não faz mais nada
cmd_display_show
        ; converter valor especificado para bcd
        MOVF    pos_value, W
        MOVWF   B1
        CLRF    B2
        CALL    SEND_DISPLAY
        RETURN

cmd_manual
        BCF     SMCON, ST_AUTO
        RETURN

;***********************************
;*
;* Comandos Manuais
;*
;**********************************

; para o movimento ficar suave manter o tempo total desta execução em 4ms
cmd_open_all
        BSF     SMCON, ST_MOVE_ALL
cmd_open
        BTFSS   PORTA, RA0
        BCF     SMCON, ST_MOVE_ALL

        call    MOTOR_DIRECT

        call    DELAYms
        call    DELAYms
        call    DELAYms
        call    DELAYms

        BTFSC   SMCON, ST_MOVE_ALL
        GOTO    cmd_open

        ; enquanto não terminar os tres envios fica em loop
        MOVF    IrDA_discard, F
        btfss   STATUS, Z
        goto    cmd_open

        RETURN

; para o movimento ficar suave manter o tempo total desta execução em 4ms
cmd_close_all
        BSF     SMCON, ST_MOVE_ALL
cmd_close
        BTFSS   PORTA, RA1
        BCF     SMCON, ST_MOVE_ALL

        call    MOTOR_REVERSE

        call    DELAYms
        call    DELAYms
        call    DELAYms
        call    DELAYms

        BTFSC   SMCON, ST_MOVE_ALL
        GOTO    cmd_close

        ; enquanto não terminar os tres envios do controle fica em loop
        MOVF    IrDA_discard, F
        btfss   STATUS, Z
        goto    cmd_close

        RETURN

cmd_auto
        BSF     SMCON, ST_AUTO        ; coloca em modo automático
        RETURN


    END                         ; directive 'end of program'
