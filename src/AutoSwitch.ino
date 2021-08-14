// AutoSwitch v2
// rene-d 08/2021

// #define ENABLE_TRACE
// #define ENABLE_VERBOSE

/*
 * constants
 */
const uint8_t LED_PIN = 4;                  // LED_BUILTIN (use a 220Ω resistor in series with the LED)
const uint8_t ACS712_SENSOR_PIN = A4;       // ACS712 sensor plugged to analog pin A4
const uint8_t RELAY_PIN = 2;                // Relay module plugged to digital pin D2
const uint8_t DELAY_POTENTIOMETER_PIN = A1; // Potentiometer connected to analog pin A1
/*
    how to connect the potentiometer to the Arduino

    Vcc ---[ 100 kΩ ]--- GND
               ➚
               o----- A1
*/
const uint8_t BYPASS_BUTTON_PIN = 3; // Bypass button to control the vacuum
/*
   how to connect the 10kΩ pull-down resistor

   D3 --+--[ 10kΩ ]---- GND
        |
        |   __L_
        +---o  o----- Vcc
*/

const int SENSITIVITY = 100;                    // Sensor sensitivity from datasheet in mV/A. 5A sensor=185, 20A=100, 30A=66
const int ACS712_THRESHOLD = 20;                // Threshold to trigger the relay
const unsigned long SAMPLE_PERIOD = 250;        // Duration of the sampling period of th ACS712 in ms
const unsigned long RELAY_ON_DELAY = 500;       // Delay before turning on the relay in ms
const unsigned long RELAY_OFF_DELAY_MIN = 1000; // Min delay to turn off the relay in ms after the tool is switched off
const unsigned long RELAY_OFF_DELAY_MAX = 5000; // Max delay to turn off the vacuum
const int BYPASS_THRESHOLD = 20;                // Treshold to activate the bypass mode

/*
 * serial trace class
 */
#ifdef ENABLE_TRACE
class
{
    bool enabled_ = true;

public:
    void begin(unsigned long baud)
    {
        Serial.begin(baud);
    }

    template <typename T>
    void print(T t)
    {
        if (enabled_)
            Serial.print(t);
    }

    template <typename T, typename U>
    void print(T t, U u)
    {
        if (enabled_)
            Serial.print(t, u);
    }

    template <typename T>
    void println(T t)
    {
        if (enabled_)
            Serial.println(t);
    }
} trace;
#else
class
{
public:
    void begin(...) {}
    void print(...) {}
    void println(...) {}
} trace;
#endif

/*
 *  global variables
 */
unsigned long trigger_deadline = 0; // Date in milliseconds of the next change
bool vacuum_state = false;          // Relay state in the next change
bool tool_state = false;            // false: tool is off, true: tool is on
bool button_state = false;          // Bypass button state

/*
 * routines
 */

// https://www.instructables.com/id/Secret-Arduino-Voltmeter/
long readVcc()
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2);            // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC))
        ; // measuring

    uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result;              // Vcc in millivolts
}

// read the switch delay in milliseconds
unsigned long get_relay_off_delay()
{
    unsigned long ms;

    ms = RELAY_OFF_DELAY_MIN + ((RELAY_OFF_DELAY_MAX - RELAY_OFF_DELAY_MIN) * analogRead(DELAY_POTENTIOMETER_PIN)) / 1023;

    trace.print("Relay off delay: ");
    trace.print(ms);
    trace.println(" ms");

    return ms;
}

// turn on or off the vacuum relay
void set_relay()
{
    trace.print("Relay set to ");
    trace.println(vacuum_state ? "ON" : "OFF");
    digitalWrite(LED_PIN, vacuum_state ? HIGH : LOW);
    digitalWrite(RELAY_PIN, vacuum_state ? LOW : HIGH);
}

// read the sensor state during X ms to determine the highest value
void read_acs712(int &value, int &bypass_ms)
{
    int n = 0;        // number of samples
    float m = 0.;     // mean value
    int min_x = 1023; // lowest value
    int max_x = 0;    // highest value

    int bypass_count = 0;

    for (auto t = millis() + SAMPLE_PERIOD; millis() < t;)
    {
        int x = analogRead(ACS712_SENSOR_PIN);
        min_x = min(min_x, x);
        max_x = max(max_x, x);
        m += x;
        n += 1;

        // also read the bypass button
        if (digitalRead(BYPASS_BUTTON_PIN) == HIGH)
            bypass_count++;
    }

    if (n == 0)
    {
        bypass_ms = 0;
        value = 0;
        return;
    }

    value = max_x - min_x;
    bypass_ms = (bypass_count * SAMPLE_PERIOD) / n;

#ifdef ENABLE_VERBOSE
    long Vcc = readVcc();
    long Vpp = (((max_x - min_x) / 2) * Vcc) / 1024L;
    long convertedmA = (707L * Vpp) / SENSITIVITY; // 1/sqrt(2) = 0.7071 = 707.1/1000

    // clang-format off
    trace.print("n="); trace.print(n);
    trace.print(" m="); trace.print(m / n, 2);
    trace.print(" d="); trace.print(max_x - min_x);
    trace.print(" ["); trace.print(min_x);
    trace.print(","); trace.print(max_x);
    trace.print("] Vcc="); trace.print(Vcc);
    trace.print(" mA="); trace.println(convertedmA);
    // clang-format on
#endif
}

/*
 * main program
 */
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BYPASS_BUTTON_PIN, INPUT);

    // relay is off by default
    set_relay();

    trace.begin(9600);
    trace.println("Vacuum AutoSwitch starting");
}

void loop()
{
    int bypass_ms = 0;
    int value = 0;
    bool new_tool_state;

    read_acs712(value, bypass_ms);
    new_tool_state = value >= ACS712_THRESHOLD;

#ifdef ENABLE_VERBOSE
    // clang-format off
    trace.print("value: "); trace.print(value);
    trace.print("  button: "); trace.println(bypass_ms);
    // clang-format on
#endif

    // hysteresis
    if (tool_state != new_tool_state)
    {
        trace.println("ACS712 changed");

        // we detected a change in the ACS712 state
        if (new_tool_state)
        {
            if (vacuum_state == false)
            {
                // transition from state off to state on
                trigger_deadline = millis() + RELAY_ON_DELAY;
                vacuum_state = true;
                trace.println("Will turn vacuum ON");
            }
        }
        else
        {
            if (vacuum_state == true)
            {
                // turn off the vacuum after a delay
                trigger_deadline = millis() + get_relay_off_delay();
                vacuum_state = false;
                trace.println("Will turn vacuum OFF");
            }
        }

        // save the new state
        tool_state = new_tool_state;
    }
    else
    {
        bool new_button_state = bypass_ms > BYPASS_THRESHOLD;
        bool button_pressed = false;

        if (new_button_state != button_state)
        {
            button_state = new_button_state;
            if (button_state)
                button_pressed = true;
        }

        if (button_pressed)
        {
            button_pressed = false;
            trace.print("Bypass button pressed: turn ");
            trace.print(vacuum_state ? "OFF" : "ON");
            trace.println(" the vacuum");

            // invert immediately the vacuum state
            vacuum_state = !vacuum_state;
            trigger_deadline = 0;
            set_relay();
        }
    }

    // set the relay
    if (trigger_deadline != 0 && trigger_deadline <= millis())
    {
        // the hysterese has expired: apply the new state to the relay
        trigger_deadline = 0;
        set_relay();
    }

    else if (trigger_deadline != 0)
    {
        // blink the led during transition phases
        digitalWrite(LED_PIN, digitalRead(LED_PIN) == HIGH ? LOW : HIGH);
    }
}
