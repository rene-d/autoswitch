// AutoSwitch
// rene-d 12/2018

//#define ENABLE_PRINT
//#define ENABLE_VERBOSE


#ifndef ENABLE_PRINT
// disable Serial output
#define Serial SomeOtherwiseUnusedName
static class {
    public:
        void begin(...) {}
        void print(...) {}
        void println(...) {}
} Serial;
#endif

/*
    constantes
*/
const uint8_t ACS712_SENSOR_PIN = A2;       // ACS712 sensor connecté à analog pin A0
const uint8_t RELAY_PIN         = 2;        // Module relais connecté à digital pin 2

const int SENSITIVITY = 100;                // sensor sensitivity from datasheet in mV/A. 5A sensor=185, 20A=100, 30A=66
const int ACS712_THRESHOLD = 30;            // seuil pour déclencher le relais
const unsigned long SAMPLE_PERIOD = 250;    // durée de la lecture du signal de l'acs712
const unsigned long RELAY_ON_DELAY = 1000;  // délai avant d'allumer l'aspirateur après le démarrage de l'outil
const unsigned long RELAY_OFF_DELAY = 3000; // délai pour éteindre l'aspirateur après l'arrêt de l'outil

/*
    variables globales
*/
unsigned long trigger_deadline = 0;     // date en ms du prochain changement
bool vacuum_state = false;              // état du relais lors du prochain changement
bool tool_state = false;                // false: outil éteint, true: outil allumé

/*
    programme
*/
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);

    // relais éteint par défaut
    set_relay();

    Serial.begin(9600);
    Serial.println("Vacuum AutoSwitch starting");
}



// https://www.instructables.com/id/Secret-Arduino-Voltmeter/
long readVcc()
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts
}


void set_relay()
{
    Serial.print("Relay set to "); Serial.println(vacuum_state ? "ON" : "OFF");
    digitalWrite(LED_BUILTIN, vacuum_state ? HIGH : LOW);
    digitalWrite(RELAY_PIN, vacuum_state ? LOW : HIGH);
}

int read_acs712()
{
    // lecture pendant 500ms de l'état du senseur pour déterminer son niveau le plus haut
    int n = 0;
    float m = 0.;       // moyenne
    int min_x = 1023;
    int max_x = 0;

    for (auto t = millis() + SAMPLE_PERIOD; millis() < t;)
    {
        int x = analogRead(ACS712_SENSOR_PIN);
        min_x = min(min_x, x);
        max_x = max(max_x, x);
        m += x;
        n += 1;
    }

    if (n == 0)
    {
        return 0;
    }

#ifdef ENABLE_VERBOSE
    long Vcc = readVcc();
    long Vpp = (((max_x - min_x) / 2) * Vcc) / 1024L;
    long convertedmA = (707L * Vpp) / SENSITIVITY;  // 1/sqrt(2) = 0.7071 = 707.1/1000

    Serial.print("n="); Serial.print(n);
    Serial.print(" m="); Serial.print(m / n, 2);
    Serial.print(" d="); Serial.print(max_x - min_x);
    Serial.print(" ["); Serial.print(min_x);
    Serial.print(","); Serial.print(max_x);
    Serial.print("] Vcc="); Serial.print(Vcc);
    Serial.print(" mA="); Serial.println(convertedmA);
#endif

    return max_x - min_x;
}


void loop()
{
    bool new_tool_state = read_acs712() >= ACS712_THRESHOLD;

    // hystérésis
    if (tool_state != new_tool_state)
    {
        // on vient de détecter un changement dans l'état de l'ACS712
        if (new_tool_state)
        {
            // on passe de l'état éteint à l'état allumé
            trigger_deadline = millis() + RELAY_ON_DELAY;
            vacuum_state = true;
            Serial.println("Will turn vacuum ON");
        }
        else
        {
            // on passe de l'état allumé à l'état éteint
            trigger_deadline = millis() + RELAY_OFF_DELAY;
            vacuum_state = false;
            Serial.println("Will turn vacuum OFF");
        }
        tool_state = new_tool_state;
    }

    // programme le relais
    if (trigger_deadline != 0 && trigger_deadline <= millis())
    {
        // la date de changement a échu: on applique le nouvel état au relais
        trigger_deadline = 0;
        set_relay();
    }

    else if (trigger_deadline != 0)
    {
        // fait clignoter la led pendant les phases de transition
        digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == HIGH ? LOW : HIGH);
    }

}
