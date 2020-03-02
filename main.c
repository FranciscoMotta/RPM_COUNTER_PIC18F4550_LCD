/*
 * File:   main.c
 * Author: HP
 *
 * Created on 21 de diciembre de 2019, 11:11 PM
 */

/*
 * File:   TECLAS.c
 * Author: HP
 *
 * Created on 16 de diciembre de 2019, 09:50 PM
 */

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS   // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
//#define ENABLE PORTCbits.RC0
//#define RS PORTCbits.RC1 
//#define PUERTO LATD
#define _XTAL_FREQ 20000000

#define LCD_RS PORTCbits.RC2
#define LCD_RW PORTCbits.RC1
#define LCD_EN PORTCbits.RC0
#define LCD_DATOS PORTD
#define ENTER 0X5A

#define ENABLE() ((LCD_EN = 1), (LCD_EN = 0))

#define BORRAR_LCD 0X01
////////////////////////////////////////////////////////////////////////////////////

void confi_puertos() {
    TRISD = 0x00;
    TRISC = 0x00;
    TRISAbits.RA4 = 1; //La entrada de reloj externa para la cuenta del timer0
}

void escribir_lcd(unsigned char dato) {
    __delay_us(40);
    LCD_DATOS = ((dato >> 4) & 0x0F); //Hacemos el corrimiento de 4 bits a la derecha y multiplicamos por 0X0F 
    ENABLE();
    LCD_DATOS = (dato & 0x0F);
    ENABLE();
}

void Borrar_lcd() {
    LCD_RS = 0; //Estamos en modo comando 
    escribir_lcd(BORRAR_LCD);
    __delay_ms(2);
}

//Sub funcion para escribir cadena caracteres especificos 

void poner_cadena(const char * cadena) {
    LCD_RS = 1; //Ponemos al LCD en modo caracter 
    while (*cadena)
        escribir_lcd(*cadena++);
}

//Sub funcion para escribir un solo caracter en el lcd

void poner_caracter(char caracter) {
    LCD_RS = 1; //Ponemos al LCD en modo caracter 
    escribir_lcd(caracter);
}

//Posiciones 

void linea_superior(unsigned char arriba) {
    LCD_RS = 0; //Ponemos al LCD en modo comando
    escribir_lcd(0x80 + arriba);
}

void linea_inferior(unsigned char abajo) {
    LCD_RS = 0; //Ponemos al LCD en modo comando
    escribir_lcd(0xC0 + abajo);
}

void lcd_inicio() {
    TRISC = 0X00;
    TRISD = 0X00;
    char valor_inicial;
    valor_inicial = 0x3;
    LCD_RS = 0;
    LCD_RW = 0;
    LCD_EN = 0;

    __delay_ms(10);
    LCD_DATOS = valor_inicial;
    ENABLE();
    __delay_ms(5);
    ENABLE();
    __delay_ms(200);
    ENABLE();
    __delay_ms(200);
    LCD_DATOS = 2; // Ponemos al LCD en modo de 4 bits de datos 
    ENABLE();
    escribir_lcd(0x28);
    escribir_lcd(0xC); //Display ON, cursor OFF, parpadeo del cursor OFF, para que se prensa = 0XF 
    Borrar_lcd(); //Limpiamos el LCD
    escribir_lcd(0x6); //Modo de entrada de datos 
}

void timer0_init(void) {
    ADCON1bits.PCFG = 0b1111; //Ponemos los pines analógos como digitales 
    T0CONbits.T08BIT = 1; //Ponemos el timer0 a trabajar a 8 bits 
    T0CONbits.T0CS = 1; //Ponemos al timer con la entrada externa de reloj 
    T0CONbits.T0SE = 1; //Ponemos la cuenta en función al flanco de bajada 
    T0CONbits.PSA = 1; //No asignamos el prescaler, dejando esta funcion al WTD
}

void main(void) {
    int unidades, decenas, centenas = 0;
    confi_puertos();
    lcd_inicio();
    linea_superior(0);
    poner_cadena("CONTADOR DE RPM");
    T0CONbits.TMR0ON = 1;
    TMR0 = 0;
    while (1) {
        centenas = TMR0 / 100 ; 
        unidades = (TMR0 %100) % 10;
        decenas = (TMR0%100) / 10;
        linea_inferior(5);
        poner_caracter(centenas + 0x30);
        poner_caracter(decenas + 0x30);
        poner_caracter(unidades + 0x30);
    }
    return;
}
