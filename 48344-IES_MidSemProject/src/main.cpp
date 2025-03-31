#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) (reg >> n & 1)
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitInverse(reg, n) (reg ^= (1 << n))

void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(char *pstr);
void usart_send_num(float num, char num_int, char num_decimal);

void usart_init_v2(float baud);
void usart_flush();
void usart_read_string(char *ptr);

void LedControl(int brightness, int T, bool status, int LED_Pin);
void Led_Dictator(int brightness, int Time);
void debugPrint();

void Setup(); 

float brightness_Definer(int ADC_Low_Light, int ADC_High_Light);

void Myteleplot(float brightness, int ADC_Low_Light, int ADC_High_Light);

void project_Debugging(int DebugState, float Sonar_Range, float Brightness);
void printMenu();


#define BUF_SIZE 50
char usart_buf[BUF_SIZE] = {0};
char *pbuf_usart = usart_buf;
volatile bool flag_read_done = 0;
volatile bool flag_interrupted = 0;

#define Pin_Red_Led     PB3  // Red
#define Pin_Yellow_Led  PB4  // Yellow
#define Pin_Green_Led   PB5  // Green

#define pin_button_change_led_mode PD2

#define pin_button_change_sys_mode PB1

int led_Mode = 0;
// 0 = green, 1 = Yellow, 2 = Red

int sys_Mode = 0;
// 0 = Auto, 1 = Manual, 2 = Emergency

int debug_State = 0;
// 0 = Nothing, 1 = Distance, 2 = Brightness, 3 = Both

int adc = 0;


ISR(USART_RX_vect)
{
    if (flag_read_done)
        return;

    flag_interrupted = 1;
    // usart_send_string("interrupted :)\r\n");
    usart_read_string(pbuf_usart);
    // usart_flush();
    // _delay_ms(50);
}

ISR(INT0_vect)
{
    _delay_ms(10);
    usart_send_string("LED INTERUPT INT0\r\n");
    if (bitRead(PIND, pin_button_change_led_mode))
    {
        led_Mode = (led_Mode + 1) % 3;
    }
}
// int counter = 0;
ISR(PCINT0_vect)
{   
    _delay_ms(10);
    if (bitRead(PINB, pin_button_change_sys_mode))
    {
        usart_send_string("SYS INTERUPT PCINT0_vect\r\n");
        sys_Mode = (sys_Mode + 1) % 3;
    }
}

ISR(ADC_vect){
  adc = ADC;
}


int main(void)
{
    Setup();
    
    usart_init_v2(19200);
    _delay_ms(100);
    usart_flush();  

    sei();

    bitSet(ADCSRA, ADSC);

    int ADC_Low_Light = 0;
    int ADC_High_Light = adc;
    
    
    int counter=0;
    float Distance = 0;
    printMenu();
    usart_flush();
    while (1)
    {
        counter = (counter+1)%180;
        Distance = sin(counter*3.14/30);

        // Constantly adjust scale for brightness 
        bitSet(ADCSRA, ADSC);
        if (ADC_Low_Light == 0 && ADC_High_Light != 0)
        {
            ADC_Low_Light = ADC_High_Light;
        }
        

        if (adc < ADC_Low_Light && adc != 0)
        {
            ADC_Low_Light = adc; // Constantly adjust scale for brightness
        }
        
        if (adc > ADC_High_Light)
        {
            ADC_High_Light = adc;
        }

        if (flag_read_done)
        {

            usart_flush();
            flag_interrupted = 0;
            debug_State = atoi(usart_buf);
            flag_read_done = 0;

            usart_send_string("Your Debug Choice: ");
            usart_send_string(usart_buf);
            usart_send_string("\r\n");
            
            printMenu();
            usart_flush();

        }
        
        // usart_send_string("test");
        project_Debugging(debug_State, Distance, brightness_Definer(ADC_Low_Light,ADC_High_Light));    
        

        // _delay_ms(100);
        switch (sys_Mode)
        {
        case 0:
            // Auto Mode
            led_Mode = (led_Mode+1)%3;
            break;

        case 1:
            // Manual Mode
            break;

        case 2:
            led_Mode = 0;

            break;

        default:
            break;
        }

        Led_Dictator(255*brightness_Definer(ADC_Low_Light,ADC_High_Light),1000);


        // debugPrint();
        // Myteleplot(brightness_Definer(ADC_Low_Light, ADC_High_Light), ADC_Low_Light, ADC_High_Light);
        
        usart_flush();
    }
}

void LedControl(int brightness, int T, bool status, int LED_Pin)
{
    /*
    Brightness 0 - 256
    T time in ms
    status: 1 / 0 -> on or off
    LED_Pin
    */

    volatile int Toff = 256 - brightness;
    volatile int Ton = 256 - Toff;

    if (status && brightness != 0)
    {
        for (int j = 0; j < T * 1000; j++)
        {

            bitSet(PORTB, LED_Pin);
            for (int i = 0; i < Ton; i++)
            {
                _delay_us(1);
            }
            bitClear(PORTB, LED_Pin);
            for (int i = 0; i < Toff; i++)
            {
                _delay_us(1);
            }
        }
    }
    else
    {
        bitClear(PORTB, LED_Pin);
    }
}

void Led_Dictator(int brightness, int Time)
{
    switch (led_Mode)
    {
    case 0:
        // Ensure all LED's are off
        LedControl(brightness, 0, 0, Pin_Yellow_Led);
        LedControl(brightness, 0, 0, Pin_Green_Led);
        LedControl(brightness, 0, 0, Pin_Red_Led);

        // Turn on GREEN LED
        LedControl(brightness*0.25, 1, Time, Pin_Green_Led);
        break;

    case 1:
        // Ensure all LED's are off
        LedControl(brightness, 0, 0, Pin_Yellow_Led);
        LedControl(brightness, 0, 0, Pin_Green_Led);
        LedControl(brightness, 0, 0, Pin_Red_Led);

        // Turn on YELLOW LED
        LedControl(brightness, 1, Time, Pin_Yellow_Led);
        break;

    case 2:
        // Ensure all LED's are off
        LedControl(brightness, 0, 0, Pin_Yellow_Led);
        LedControl(brightness, 0, 0, Pin_Green_Led);
        LedControl(brightness, 0, 0, Pin_Red_Led);

        // Turn on RED LED
        LedControl(brightness, 1, Time, Pin_Red_Led);
        break;

    default:
        break;
    }
}

void debugPrint()
{
    usart_send_string("----------------------\r\n");

    // usart_send_num(sys_Mode, 0, 0);
    switch (sys_Mode)
    {
    case 0:
            usart_send_string("SYS_MODE: Auto");
        break;
    
    case 1:
            usart_send_string("SYS_MODE: Manual");
        break;

    case 2:
            usart_send_string("SYS_MODE: Emergency");
        break;

    default:
        break;
    }

    usart_send_string("\r\n");

    usart_send_string("\r\n");

    switch (led_Mode)
    {
    case 0:
            usart_send_string("Led: Green");
        break;
    
    case 1:
            usart_send_string("Led: Yellow");
        break;

    case 2:
            usart_send_string("Led: Red");
        break;

    default:
        break;
    }


    usart_send_string("\r\n");
}

float brightness_Definer(int ADC_Low_Light, int ADC_High_Light){
    float brightness = adc;
    brightness = (brightness - ADC_Low_Light)/(ADC_High_Light - ADC_Low_Light);
    if (brightness<0.1)
    {
        return 0.1;
    }
    
    return brightness;
}

void Myteleplot(float brightness, int ADC_Low_Light, int ADC_High_Light){
        usart_send_string(">brightness:");
        usart_send_num(brightness, 4, 4);
        usart_send_string("\n");

        // usart_send_string(">ADC_Low_Light:");
        // usart_send_num(ADC_Low_Light, 4, 4);
        // usart_send_string("\n");

        // usart_send_string(">ADC_High_Light:");
        // usart_send_num(ADC_High_Light, 4, 4);
        // usart_send_string("\n");

        // usart_send_string(">adc:");
        // usart_send_num(adc, 4, 4);
        // usart_send_string("\n");
}

void printMenu(){
    usart_send_string("Debug Menu: \r\n");
    
    usart_send_string("0: No Debug Graphs. \r\n");

    usart_send_string("1: Distance Graph. \r\n");

    usart_send_string("2: Brightness Graph. \r\n");

    usart_send_string("3: Brighness and Distance Graph. \r\n");
    usart_send_string("Enter Debug Choice: \r\n");
}


void project_Debugging(int DebugState, float Sonar_Range, float Brightness){
    switch (DebugState)
    {
    case 0:
        /* Nothing */
        break;
    case 1:
        /* Distance Measurements */
        usart_send_string(">Distance:");
        usart_send_num(Sonar_Range, 4, 4);
        usart_send_string("\n");
        break;
    case 2:
        /* Ambient Brightness || LED Brightness */
        usart_send_string(">Brightness:");
        usart_send_num(Brightness, 4, 4);
        usart_send_string("\n");
        break;
    case 3:
        /* Both */

        /* Ambient Brightness || LED Brightness */
        usart_send_string(">Brightness:");
        usart_send_num(Brightness, 4, 4);
        usart_send_string("\n");

        usart_send_string(">Distance:");
        usart_send_num(Sonar_Range, 4, 4);
        usart_send_string("\n");

        break;

    default:
        break;
    }
}

void Setup(){

    bitSet(DDRB, Pin_Red_Led);
    bitSet(DDRB, Pin_Green_Led);
    bitSet(DDRB, Pin_Yellow_Led);

        
    bitClear(DDRD,pin_button_change_led_mode);
    bitSet(PORTD,pin_button_change_led_mode);
        
    bitClear(DDRB,pin_button_change_sys_mode);
    bitSet(PORTB,pin_button_change_sys_mode);


    EIMSK |= 1 << INT0;
    EICRA |= 3 << 0;

    bitSet(PCICR,PCIE0);
    bitSet(PCMSK0,PCINT1);

    bitSet(ADMUX, REFS0); 
    bitSet(ADMUX, MUX1); 
    ADCSRA |= 0b111 < 0;
    bitSet(ADCSRA, ADIE);
    bitSet(ADCSRA, ADEN);

}

//------------------------------------------------------------------------------

void usart_init(float baud)
{
    float ubrr0 = 1e6 / baud - 1;
    int ubrr0a = (int)ubrr0;

    if (ubrr0 - ubrr0a >= 0.5)
    {
        ubrr0a = ubrr0a + 1;
    }

    UBRR0 = ubrr0a;
    bitSet(UCSR0B, TXEN0);
    UCSR0C |= 3 << UCSZ00;
}

void usart_send_byte(unsigned char data)
{
    while (!bitRead(UCSR0A, UDRE0))
        ;
    UDR0 = data;
}

void usart_send_string(char *pstr)
{
    while (*pstr != '\0')
    {
        usart_send_byte(*pstr);
        pstr++;
    }
}

void usart_send_num(float num, char num_int, char num_decimal)
{
    char str[20];
    if (num_decimal == 0)
    {
        dtostrf(num, num_int, num_decimal, str);
    }
    else
    {
        dtostrf(num, num_int + num_decimal + 1, num_decimal, str);
    }
    str[num_int + num_decimal + 1] = '\0';
    usart_send_string(str);
}

void usart_init_v2(float baud)
{
    usart_init(baud);
    bitSet(UCSR0B, RXEN0);
    bitSet(UCSR0B, RXCIE0); // Enable USART Receive Complete interrupt
}

void usart_flush(void)
{
    char dummy;
    while (bitRead(UCSR0A, RXC0))
    {
        dummy = UDR0;
    }
}

void usart_read_string(char *ptr)
{
    char tmp;
    while (1)
    {
        while (!bitRead(UCSR0A, RXC0))
            ;
        tmp = UDR0;
        if (tmp == '\r' || tmp == '\n') {
        // if (tmp == '\n'){

            *ptr = '\0';
            ptr = usart_buf;
            flag_read_done = 1;
            return;
        }
        else
        {
            *ptr = tmp;
            ptr++;
        }
    }
}
