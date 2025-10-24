/**
 * **ATENÇÃO:** quem remover a documentação tornará-se corno dentro de 1 ano.
 * Se for solteiro, vai começar a namorar em 6 meses e será traído nos 6
 * restantes.
 *
 * Se apagar a documentação na intenção de conseguir um namoro em 6 meses, a
 * pena muda para mais 5 anos solteiro.
 *
 * @author Frank H. Borsato
 * @author João G. Wolf
 * @author Kaio F. C. Amorim
 * @author Marco A. C. Fragata.
 */

#include <Arduino.h>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////////
// PINS ALIASES
////////////////////////////////////////////////////////////////////////////////////

/// 4 bits de dados do LCD no PORTD
#define DADOS_LCD PORTD

/**
 * Essa flag indica se os pinos que recebem dados no LCD são os 4 LSB⁽¹⁾ (Px0-D4
 * a Px3-D7) ou os 4 MSB⁽²⁾ (Px4-D4 a Px7-D7).
 *
 * ---
 *
 * 1: Less Significant Bits (bits menos significativos, isso é, os mais à
 * direita).
 * 2: Most Significant Bits (bits mais significativos, isso é, os mais
 * à esquerda).
 * Obs.: 1 nibble é meio byte (ou 4 bits).
 */
#define LCD_DATA_NIBBLE 1

/// PORT com os pinos de controle do LCD (pino R/W e Enable).
#define CONTR_LCD PORTB

/// Pino de habilitação do LCD (enable)
#define LCD_ENABLE PB4

/**
 * Pino que informar se o dado é uma instrução ou caractere
 * - 0: indica que é uma instrução;
 * - 1: indica que é um caractere.
 */
#define DATA_TYPE_PIN PB3

////////////////////////////////////////////////////////////////////////////////////
// MACROS
////////////////////////////////////////////////////////////////////////////////////

/// Coloca em 1 o `i`-nésimo bit da variável `y`.
#define set_bit(y, i) (y |= (1 << i))

/// Coloca em 0 o bit `i`-nésimo bit da variável `y`.
#define clr_bit(y, i) (y &= ~(1 << i))

/// Inverte o estado lógico do `i`-nésimo bit da variável `y`.
#define cpl_bit(y, i) (y ^= (1 << i))

/// Retorna 0 ou 1 conforme leitura do `i`-ésimo da variável `y`.
#define tst_bit(y, i) (y & (1 << i))

#if (LCD_DATA_NIBBLE == 1)
#define set_most_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & data);

#define set_less_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (data << 4));
#else
#define set_most_significant_nibble(trimmed_data_nibble)                       \
  DADOS_LCD = (DADOS_LCD & 0xF0) | (data >> 4);

#define set_less_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & data);
#endif

////////////////////////////////////////////////////////////////////////////////////
// ENUMS
////////////////////////////////////////////////////////////////////////////////////

enum class MessageType
{
  Instruction,
  Character
};

enum class AvailablePotentiometer
{
  Left = 0,
  Right = 1
};

////////////////////////////////////////////////////////////////////////////////////
// Classes Declarations
////////////////////////////////////////////////////////////////////////////////////

class LcdFacade
{
public:
  enum class LcdLine
  {
    Upper,
    Lower
  };

  /**
   * Envia um comando para o LCD.
   */
  static void send_command(const uint8_t command_byte);
  /**
   * Envia um caractere para ser impresso no LCD.
   */
  static void send_character(const uint8_t character_byte);
  /**
   * Envia uma string para ser impressa no LCD.
   */
  static void send_string(const char *str);
  /**
   * Inicializa o LCD configurado para usar uma via de dados de 4 bits.
   */
  static void initialize_lcd();
  /**
   * Limpa o LCD.
   */
  static void clear();
  static void turn_cursor_on();
  static void turn_cursor_off();
  /**
   * Move o cursor para a linha `line`, a `offset` colunas após a
   * primeira coluna.
   */
  static void move_cursor(const LcdLine line, const uint8_t offset);

private:
  /**
   * Envia caracteres ou comandos para o LCD utilizando a via de 4 bits (1
   * nibble).
   */
  static void send_message(const uint8_t data, const MessageType msg_type);
  /**
   * Coloca o pino `DATA_TYPE_PIN` no respectivo estado de acordo com o tipo de
   * dado da mensagem.
   */
  static void set_message_data_type_to_pin(const MessageType msg_type);
  /**
   * Coloca os dados (`data`) no LCD nibble-a-nibble.
   */
  static void set_data_to_lcd(const uint8_t data);
  /**
   * Manda um sinal de habilitação (pulso) para o LCD.
   */
  static void enable_pulse();
};

class PropellersBase
{
private:
  uint8_t propellers;

public:
  PropellersBase();
  /**
   * Aumenta a quantidade de hélices na base, seguindo — ciclicamente —
   * adiante no intervalo [1, 9].
   */
  void increase_propellers_saturating();
  /**
   * Obtém a atual quantidade de hélices disponíveis na base.
   */
  uint8_t get_propellers() const;
};

class PinManager
{
private:
  const uint8_t pin_;
  const volatile uint8_t *register_;

public:
  PinManager(volatile uint8_t *register_, uint8_t pin_);
  /**
   * Obtém o nível digital do pino interpretado como booleano, onde:
   * - `LOW` = `false`,
   * - `HIGHT` = `true.
   */
  bool get_digital_level() const;
};

class Button
{
private:
  const PinManager pin_manager;
  // Inicializa ambos como não pressionados.
  // Lembre-se que os botões, quando pressionados, ficam no estado
  // lógico/digital `LOW` (`false`).
  bool previous_state = true;
  bool current_state = true;

public:
  Button(PinManager pin_manager);
  /**
   * Checa se houve alguma alteração no estado do botão.
   */
  bool has_changed() const;
  /**
   * Se, e somente se, `has_changed() == true`, este método retorna:
   * - `true` se o botão foi pressionado;
   * - `false` se o botão foi solto.
   *
   * Caso contrário, o resultado desta função não tem significado.
   */
  bool has_been_pressed() const;
  /**
   * Checa o nível digital do botão para checar se houve interação.
   */
  void check();
};

class LineFollowerSensor
{
private:
  float frequency = 0.0;

public:
  volatile uint32_t pulses_counter = 0;
  volatile uint32_t last_two_pulses_delta = 0;
  volatile uint32_t last_pulse_micros = 0;

  struct Statistics
  {
    uint32_t count;
    uint32_t delta;
  };

  /**
   * Extract statistics and reset sensor data.
   */
  Statistics take_statistics();
  /**
   * Measures the frequency. This is expected to be called within 100ms
   * intervals.
   */
  float measure_frequency(float old_frequency, unsigned long now);
};

////////////////////////////////////////////////////////////////////////////////////
// SETUP, LOOP & ARDUINO INTERRUPTIONS
////////////////////////////////////////////////////////////////////////////////////

/**
 * Display de frequency (in Hz) in LCD.
 */
void display_frequency(const double frequency, const char *measurement);
/**
 * Display the amount of propellers being used in LCD.
 */
void display_propellers(const uint8_t quantity);

////////////////////////////////////////////////////////////////////////////////////
// GLOBAL STATIC CONSTANTS
////////////////////////////////////////////////////////////////////////////////////

const uint8_t lcd_refresh_rate = 100;                     // ms
const uint8_t button_read_refresh_rate = 0;               // ms
const uint16_t line_follower_frequency_check_rate = 1000; // 1s
const uint8_t statisfying_measurement_limit = 10;         // ms

////////////////////////////////////////////////////////////////////////////////////
// SETUP, LOOP & ARDUINO INTERRUPTIONS
////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Torna os primeiros 4 pinos do registrador D saídas (são saídas do LCD).
  DDRD |= 0b11110000;

  // Torna os pinos que controlam o LCD em saídas (conectados no Enable e
  // Read/Write). Torna o pino do botão uma entrada.
  DDRB |= 0b00011000;

  // Ativa o pull-up do botão
  PORTB |= (1 << PB0);

  // DIDRx: Digital Input Disable Register x
  // Um único pino no ATmega328 pode ser uma saída ou uma entrada (digital ou
  // analógica); ele é multifuncional.
  //
  // A funcionalidade de entrada digital (um amplificador conectado no pino pra
  // realizar a leitura digital) pode criar um ruído que atrapalha a leitura da
  // entrada analógica. Por isso, desligamos o circuito que faz a leitura
  // digital no pino que será lido para evitar que ele gere ruído e atrapalhe a
  // leitura analógica.
  //
  // Observe que:
  // Quando um bit em DIDRx é:
  // * 0, então o buffer digital está ligado
  // * 1, então o buffer digital está desligado
  //
  // Aqui, desligamos o buffer digital de todos os pinos da porta C (de inputs
  // analógicos). Podemos desligar todos pois somente o terceiro pino está
  // sendo utilizado, e é o do sensor segue faixa. Portanto, não afetamos nenhum
  // outro periférico deste pino.
  DIDR0 = 0b00111111;

  // TODO: enable what interrups?
  // falling edge??

  // Ativa o pull-up do sensor segue faixa.
  PORTC |= (1 << PC3);
  // Permite que mudanças no estado deses pino enganchem interrupções.
  PCMSK1 |= (1 << PC3);
  // Habilita interrupções para o grupo de interrupções 1 (A0 - A5).
  PCICR |= (1 << PCIE1);
  // Limpa a flag de interrupção para caso tenha alguma pendente de outra
  // execução anterior.
  PCIFR |= (1 << PCIF1);
  // Força o arduino a parar o loop diante de uma interrupção, mesmo que
  // configuradas manualmente (como é o caso aqui).
  interrupts();

  LcdFacade::initialize_lcd();
  LcdFacade::turn_cursor_off();
}

auto line_follower_sensor = LineFollowerSensor();

void loop()
{
  static auto propellers_base = PropellersBase();
  static auto button = Button(PinManager(&PINB, PB0));

  button.check();

  static size_t last_lcd_update = 0;
  static size_t last_button_read = 0;
  static size_t last_sensor_check = 0;
  static auto pulse_frequency = 0.0;

  auto now = millis();
  if (now - last_lcd_update > lcd_refresh_rate)
  {
    last_lcd_update = now;

    pulse_frequency =
        line_follower_sensor.measure_frequency(pulse_frequency, now);

    /** Frequência de rotação do eixo em HZ/RPS */
    const auto estimated_axis_frequency =
        pulse_frequency / propellers_base.get_propellers();

    display_frequency(estimated_axis_frequency, "Hz");
    display_propellers(propellers_base.get_propellers());
  }

  if (button.has_changed() &&
      (now - last_button_read > button_read_refresh_rate))
  {
    last_button_read = now;
    if (button.has_been_pressed())
    {
      propellers_base.increase_propellers_saturating();
    }
  }

  if (now - last_sensor_check > line_follower_frequency_check_rate)
  {
    last_sensor_check = now;
  }
}

// Como só o pino A3 está conectado no grupo 1, é garantido
// que essa interrupção só será acionada diante de alterações
// no valor lógico do pino A3.
ISR(PCINT1_vect)
{
  static auto pin_manager = PinManager(&PINC, PC3);

  // Executa somente quando estiver no sinal baixo (falling edge).
  if (pin_manager.get_digital_level()) return;

  auto now_micros = micros();

  const auto has_previous_pulse = line_follower_sensor.last_pulse_micros > 0;
  if (has_previous_pulse)
  {
    line_follower_sensor.last_two_pulses_delta =
        now_micros - line_follower_sensor.last_pulse_micros;
  }

  line_follower_sensor.pulses_counter++;
  line_follower_sensor.last_pulse_micros = now_micros;
}

////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTAÇÕES DAS CLASSES
////////////////////////////////////////////////////////////////////////////////////

void LcdFacade::enable_pulse()
{
  set_bit(CONTR_LCD, LCD_ENABLE);
  _delay_us(1);
  clr_bit(CONTR_LCD, LCD_ENABLE);
  _delay_us(1);
  _delay_us(45);
}

void LcdFacade::set_data_to_lcd(const uint8_t data)
{
  set_most_significant_nibble(data);
  enable_pulse();
  set_less_significant_nibble(data);
  enable_pulse();
}

void LcdFacade::set_message_data_type_to_pin(const MessageType msg_type)
{
  switch (msg_type)
  {
  case MessageType::Instruction:
    clr_bit(CONTR_LCD, DATA_TYPE_PIN);
    break;
  case MessageType::Character:
    set_bit(CONTR_LCD, DATA_TYPE_PIN);
    break;
  }
}

void LcdFacade::send_message(const uint8_t data, const MessageType msg_type)
{
  set_message_data_type_to_pin(msg_type);
  set_data_to_lcd(data);

  const auto is_return_or_clean_instruction =
      msg_type == MessageType::Instruction && data < 4;

  if (is_return_or_clean_instruction) _delay_ms(2);
}

void LcdFacade::send_command(const uint8_t command_byte)
{
  send_message(command_byte, MessageType::Instruction);
}

void LcdFacade::send_character(const uint8_t character_byte)
{
  send_message(character_byte, MessageType::Character);
}

void LcdFacade::clear() { send_command(0x01); }
void LcdFacade::turn_cursor_on() { send_command(0x0F); }
void LcdFacade::turn_cursor_off() { send_command(0x0C); }

void LcdFacade::move_cursor(const LcdLine line, const uint8_t offset)
{
  switch (line)
  {
  case LcdLine::Upper:
    return send_command(0x80 + offset);
  case LcdLine::Lower:
    return send_command(0xC0 + offset);
  }
}

void LcdFacade::initialize_lcd()
{
  clr_bit(CONTR_LCD, DATA_TYPE_PIN);
  clr_bit(CONTR_LCD, LCD_ENABLE);

  _delay_ms(20);
  send_message(0x30, MessageType::Instruction);
  enable_pulse();
  _delay_ms(5);
  enable_pulse();
  _delay_ms(200);
  enable_pulse();
  send_command(0x20);
  enable_pulse();
  send_command(0x28);
  send_command(0x08);
  send_command(0x01);
  send_command(0x0F);
  send_command(0x80);
}

void LcdFacade::send_string(const char *str)
{
  size_t i = 0;
  while (str[i])
  {
    LcdFacade::send_character(str[i]);
    i++;
  }
}

void display_frequency(const double frequency, const char *measurement)
{
  const uint8_t available_pixels = 7;
  // precisa de espaço pro caractere nulo, por isso `available_pixels` + 1
  char frequency_string[available_pixels + 1];
  dtostrf(frequency, available_pixels, 1, frequency_string);

  LcdFacade::move_cursor(LcdFacade::LcdLine::Upper, 0);
  for (uint8_t i = 0; i < available_pixels; i++)
  {
    LcdFacade::send_character(frequency_string[i]);
  }

  LcdFacade::send_string(measurement);
}

PropellersBase::PropellersBase() : propellers(0) {}

void display_propellers(const uint8_t quantity)
{
  const char prefix[] = "Helices: ";
  LcdFacade::move_cursor(LcdFacade::LcdLine::Lower, 0);
  LcdFacade::send_string(prefix);

  char buffer[2];
  itoa(quantity, buffer, 10);
  LcdFacade::send_character(buffer[0]);
}

void PropellersBase::increase_propellers_saturating()
{
  ++this->propellers %= 9;
}

uint8_t PropellersBase::get_propellers() const { return this->propellers + 1; }

PinManager::PinManager(volatile uint8_t *register_, uint8_t pin_)
    : pin_(pin_), register_(register_)
{
}

bool PinManager::get_digital_level() const
{
  return ((*this->register_) & (1 << this->pin_)) != 0;
}

Button::Button(PinManager pin_manager) : pin_manager(pin_manager) {}

bool Button::has_changed() const
{
  return this->current_state != this->previous_state;
}

bool Button::has_been_pressed() const { return !this->current_state; }

void Button::check()
{
  this->previous_state = this->current_state;
  this->current_state = this->pin_manager.get_digital_level();
}

LineFollowerSensor::Statistics LineFollowerSensor::take_statistics()
{
  noInterrupts();
  Statistics statistics = {.count = this->pulses_counter,
                           .delta = this->last_two_pulses_delta};
  this->pulses_counter = 0;
  interrupts();
  return statistics;
}

float LineFollowerSensor::measure_frequency(float old_frequency,
                                            unsigned long now)
{
  auto sensor_statistics = line_follower_sensor.take_statistics();

  if (sensor_statistics.delta > statisfying_measurement_limit)
  {
    // Calcula a frequência em Hz
    const auto secondInMicros = 1000000.0;
    return secondInMicros / (float)sensor_statistics.delta;
  }

  if (sensor_statistics.count > 0)
  {
    const auto pulses_within_100_ms = sensor_statistics.count;
    const auto pulses_within_1_sec = pulses_within_100_ms * 10;
    return (float)pulses_within_1_sec;
  }

  const uint8_t inactivity_time_limit = 200; // ms
  const auto last_pulse_millis = last_pulse_micros / 1000;
  if (now - last_pulse_millis > inactivity_time_limit)
  {
    return 0.0;
  }

  return old_frequency;
}