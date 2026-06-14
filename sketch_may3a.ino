#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SigmaDSP.h>
#include "SigmaDSP_parameters.h"

SigmaDSP dsp(Wire, DSP_I2C_ADDRESS, 48000.0f, 0);

// ----------------------------------------------------------------------
// Глобальные переменные (volatile для флагов, изменяемых вне loop)
// ----------------------------------------------------------------------
volatile float currentVolume = -10.0f;
volatile bool  volumeChanged = false;

volatile float eq1_freq = 1000.0f, eq1_gain = 0.0f;
volatile float eq2_freq = 1000.0f, eq2_gain = 0.0f;
volatile bool  eq1Changed = false, eq2Changed = false;

enum Mode { MODE_VOLUME, MODE_EQ1_FREQ, MODE_EQ1_GAIN, MODE_EQ2_FREQ, MODE_EQ2_GAIN };
volatile Mode  currentMode = MODE_VOLUME;

#define ENC_CLK 14
#define ENC_DT  12
#define ENC_SW  13

const char* ssid     = "HUAWEI-1006P6";
const char* password = "qwezxcasd";
WiFiServer server(1234);
WiFiClient client;

// ----------------------------------------------------------------------
// Программный опрос энкодера (без прерываний)
// ----------------------------------------------------------------------
class RotaryEncoder {
public:
    RotaryEncoder(int pinClk, int pinDt)
        : _pinClk(pinClk), _pinDt(pinDt), _lastClk(HIGH), _lastTime(0), _direction(0)
    {
        pinMode(_pinClk, INPUT_PULLUP);
        pinMode(_pinDt, INPUT_PULLUP);
    }

    // Возвращает направление: 1 = вправо, -1 = влево, 0 = не крутили
    int read() {
        int clk = digitalRead(_pinClk);
        if (clk != _lastClk) {
            // Прошло не менее 1 мс с предыдущего изменения → настоящий поворот
            if (micros() - _lastTime > 1000) {
                if (digitalRead(_pinDt) != clk) {
                    _direction = 1;
                } else {
                    _direction = -1;
                }
                _lastClk = clk;
                _lastTime = micros();
                return _direction;
            }
        }
        return 0;
    }

private:
    int _pinClk, _pinDt;
    int _lastClk;
    unsigned long _lastTime;
    int _direction;
};

RotaryEncoder encoder(ENC_CLK, ENC_DT);

// ----------------------------------------------------------------------
// Кнопка с различением короткого/длинного нажатия
// ----------------------------------------------------------------------
class Button {
public:
    Button(int pin) : _pin(pin), _state(HIGH), _lastState(HIGH), _pressed(false),
                      _longPress(false), _pressTime(0) {
        pinMode(_pin, INPUT_PULLUP);
    }

    // Вызывать в loop() как можно чаще
    void update() {
        bool reading = digitalRead(_pin);
        if (reading == LOW && _lastState == HIGH) {
            // Нажатие началось
            _pressTime = millis();
            _pressed = true;
            _longPress = false;
        } else if (reading == LOW && _pressed && !_longPress) {
            if (millis() - _pressTime > 800) {
                _longPress = true;
            }
        } else if (reading == HIGH && _lastState == LOW) {
            // Отпускание
            if (_pressed && !_longPress && (millis() - _pressTime > 30)) {
                // Короткое нажатие
                onShortPress();
            }
            _pressed = false;
        }
        _lastState = reading;
    }

    // Эти функции будут установлены извне
    void setShortPressHandler(void (*handler)()) { _shortHandler = handler; }
    void setLongPressHandler(void (*handler)())  { _longHandler = handler; }

private:
    void onShortPress() { if (_shortHandler) _shortHandler(); }
    void onLongPress()  { if (_longHandler)  _longHandler(); }

    int _pin;
    int _state, _lastState;
    bool _pressed, _longPress;
    unsigned long _pressTime;
    void (*_shortHandler)() = nullptr;
    void (*_longHandler)()  = nullptr;
};

Button button(ENC_SW);

// Обработчики кнопки
void shortPressHandler() {
    static const char* modeNames[] = {"VOLUME","EQ1_FREQ","EQ1_GAIN","EQ2_FREQ","EQ2_GAIN"};
    // Циклическое переключение режимов
    currentMode = static_cast<Mode>((currentMode + 1) % 5);
    Serial.print("Mode: ");
    Serial.println(modeNames[currentMode]);
}

void longPressHandler() {
    static bool muted = false;
    muted = !muted;
    uint8_t cmd[2] = {0x00, muted ? 0x18 : 0x14};
    dsp.writeRegister(0x81C, 2, cmd);
    Serial.println(muted ? "MUTE" : "UNMUTE");
}

// ----------------------------------------------------------------------
// Применение эквалайзера (один фильтр)
// ----------------------------------------------------------------------
void applyEQ(int filter, float freq, float gain) {
    firstOrderEQ eq;
    if (filter == 1) {
        eq1_freq = freq;
        eq1_gain = gain;
        eq.filterType = parameters::filterType::lowpass; // только lowpass для простоты
        eq.freq = eq1_freq;
        eq.gain = eq1_gain;
        eq.state = parameters::state::on;
        dsp.EQfirstOrder(MOD_GEN1STORDER1_ALG0_PARAMB00_ADDR, eq);
    } else {
        eq2_freq = freq;
        eq2_gain = gain;
        eq.filterType = parameters::filterType::lowpass;
        eq.freq = eq2_freq;
        eq.gain = eq2_gain;
        eq.state = parameters::state::on;
        dsp.EQfirstOrder(MOD_GEN1STORDER2_ALG0_PARAMB00_ADDR, eq);
    }
    Serial.printf("EQ%d set: freq=%.0f Hz, gain=%.1f dB\n", filter, freq, gain);
}

// ----------------------------------------------------------------------
// Установка громкости (slew)
// ----------------------------------------------------------------------
void setVolume(float dB) {
    dsp.volume_slew(MOD_SWVOL1_ALG0_TARGET_ADDR, dB, 12);
    Serial.printf("Volume: %.1f dB\n", dB);
}

// ----------------------------------------------------------------------
// Инициализация DSP
// ----------------------------------------------------------------------
void setupDSP() {
    dsp.begin();
    dsp.i2cClock(100000);
    delay(2000);
    loadProgram(dsp);
    // Выключаем оба EQ на старте
    firstOrderEQ eqOff;
    eqOff.state = parameters::state::off;
    dsp.EQfirstOrder(MOD_GEN1STORDER1_ALG0_PARAMB00_ADDR, eqOff);
    dsp.EQfirstOrder(MOD_GEN1STORDER2_ALG0_PARAMB00_ADDR, eqOff);
    Serial.println("DSP loaded");
}

// ----------------------------------------------------------------------
// Основная программа
// ----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Wire.begin(4, 5);
    Wire.setClock(100000);

    setupDSP();
    setVolume(currentVolume);

    button.setShortPressHandler(shortPressHandler);
    button.setLongPressHandler(longPressHandler);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi: " + WiFi.localIP().toString());
    server.begin();
    Serial.println("Готово. Энкодер: громкость/эквалайзер, кнопка: смена режима (коротко) / Mute (длинно)");
}

void loop() {
    // 1. Опрос кнопки
    button.update();

    // 2. Опрос энкодера (направление -1/0/+1)
    int encDir = encoder.read();
    if (encDir != 0) {
        float delta = (encDir > 0) ? 1.0f : -1.0f;
        switch (currentMode) {
            case MODE_VOLUME:
                currentVolume += delta;
                if (currentVolume > 12.0f) currentVolume = 12.0f;
                if (currentVolume < -70.0f) currentVolume = -70.0f;
                volumeChanged = true;
                break;
            case MODE_EQ1_FREQ:
                eq1_freq *= (delta > 0) ? 1.1f : 0.9f;
                if (eq1_freq > 20000.0f) eq1_freq = 20000.0f;
                if (eq1_freq < 20.0f) eq1_freq = 20.0f;
                eq1Changed = true;
                break;
            case MODE_EQ1_GAIN:
                eq1_gain += delta;
                if (eq1_gain > 15.0f) eq1_gain = 15.0f;
                if (eq1_gain < -15.0f) eq1_gain = -15.0f;
                eq1Changed = true;
                break;
            case MODE_EQ2_FREQ:
                eq2_freq *= (delta > 0) ? 1.1f : 0.9f;
                if (eq2_freq > 20000.0f) eq2_freq = 20000.0f;
                if (eq2_freq < 20.0f) eq2_freq = 20.0f;
                eq2Changed = true;
                break;
            case MODE_EQ2_GAIN:
                eq2_gain += delta;
                if (eq2_gain > 15.0f) eq2_gain = 15.0f;
                if (eq2_gain < -15.0f) eq2_gain = -15.0f;
                eq2Changed = true;
                break;
        }
    }

    // 3. Применение изменений с ограничением частоты (не чаще 30 мс)
    static unsigned long lastApply = 0;
    if (millis() - lastApply >= 30) {
        if (volumeChanged) {
            volumeChanged = false;
            setVolume(currentVolume);
        }
        if (eq1Changed) {
            eq1Changed = false;
            applyEQ(1, eq1_freq, eq1_gain);
        }
        if (eq2Changed) {
            eq2Changed = false;
            applyEQ(2, eq2_freq, eq2_gain);
        }
        lastApply = millis();
    }

    // 4. Wi-Fi команды
    if (server.hasClient()) {
        if (client) server.available().stop();
        else client = server.available();
    }
    if (client && client.connected()) {
        String s = client.readStringUntil('\n');
        if (s.length() > 0) {
            DynamicJsonDocument d(512);
            if (!deserializeJson(d, s)) {
                const char* cmd = d["cmd"];
                if (cmd && !strcmp(cmd, "set_gain")) {
                    currentVolume = d["db"];
                    volumeChanged = true;
                    client.println("{\"status\":\"ok\"}");
                }
                else if (cmd && !strcmp(cmd, "set_eq")) {
                    int filter = d["filter"] | 1;
                    float freq = d["freq"] | 1000.0f;
                    float gain = d["gain"] | 0.0f;
                    if (filter == 1) { eq1_freq = freq; eq1_gain = gain; eq1Changed = true; }
                    else             { eq2_freq = freq; eq2_gain = gain; eq2Changed = true; }
                    client.println("{\"status\":\"ok\"}");
                }
            }
        }
    }
}