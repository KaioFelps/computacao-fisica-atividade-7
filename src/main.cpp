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
  static void send_command(uint8_t command_byte);
  /**
   * Envia um caractere para ser impresso no LCD.
   */
  static void send_character(uint8_t character_byte);
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
  static void move_cursor(LcdLine line, uint8_t offset);

private:
  /**
   * Envia caracteres ou comandos para o LCD utilizando a via de 4 bits (1
   * nibble).
   */
  static void send_message(uint8_t data, MessageType msg_type);
  /**
   * Coloca o pino `DATA_TYPE_PIN` no respectivo estado de acordo com o tipo de
   * dado da mensagem.
   */
  static void set_message_data_type_to_pin(MessageType msg_type);
  /**
   * Coloca os dados (`data`) no LCD nibble-a-nibble.
   */
  static void set_data_to_lcd(uint8_t data);
  /**
   * Manda um sinal de habilitação (pulso) para o LCD.
   */
  static void enable_pulse();
};

////////////////////////////////////////////////////////////////////////////////////
// GLOBAL STATIC CONSTANTS
////////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Representa qual dígito vai ser ativado na corrente iteração do`loop`.
 */
uint8_t active_digit = 0;

////////////////////////////////////////////////////////////////////////////////////
// SETUP, LOOP & ARDUINO INTERRUPTIONS
////////////////////////////////////////////////////////////////////////////////////

unsigned long long mymillis;

void setup()
{
  // Torna os primeiros 4 pinos do registrador D saídas (são saídas do LCD).
  DDRD |= 0b11110000;

  // Torna os pinos que controlam o LCD em saídas (conectados no Enable e
  // Read/Write).
  DDRB |= 0b00011000;

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

  LcdFacade::initialize_lcd();

  LcdFacade::move_cursor(LcdFacade::LcdLine::Upper, 15);
  LcdFacade::send_character(0x25);
  LcdFacade::move_cursor(LcdFacade::LcdLine::Lower, 15);
  LcdFacade::send_character(0x25);

  LcdFacade::move_cursor(LcdFacade::LcdLine::Upper, 0);
  LcdFacade::send_character(0);
  LcdFacade::move_cursor(LcdFacade::LcdLine::Lower, 0);
  LcdFacade::send_character(6);

  // LcdFacade::clear();

  // TODO: enable what interrups?
  // falling edge??
}

const uint8_t lcd_refresh_rate = 100;         // ms
const uint8_t button_read_refresh_rate = 125; // ms

void loop()
{
  static size_t last_lcd_update = 0;
  static size_t last_button_read = 0;

  auto now = millis();
  if (now - last_lcd_update > lcd_refresh_rate)
  {
    last_lcd_update = now;
    // TODO: update LCD
  }

  // TODO: ler botao
  if (now - last_button_read > button_read_refresh_rate)
  {
    last_button_read = now;
  }
}

// ISR(????_vect)
// {
// TODO: what to do?
// }

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

void LcdFacade::set_data_to_lcd(uint8_t data)
{
  set_most_significant_nibble(data);
  enable_pulse();
  set_less_significant_nibble(data);
  enable_pulse();
}

void LcdFacade::set_message_data_type_to_pin(MessageType msg_type)
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

void LcdFacade::send_message(uint8_t data, MessageType msg_type)
{
  set_message_data_type_to_pin(msg_type);
  set_data_to_lcd(data);

  const auto is_return_or_clean_instruction =
      msg_type == MessageType::Instruction && data < 4;

  if (is_return_or_clean_instruction) _delay_ms(2);
}

void LcdFacade::send_command(uint8_t command_byte)
{
  send_message(command_byte, MessageType::Instruction);
}

void LcdFacade::send_character(uint8_t character_byte)
{
  send_message(character_byte, MessageType::Character);
}

void LcdFacade::clear() { send_command(0x01); }
void LcdFacade::turn_cursor_on() { send_command(0x0F); }
void LcdFacade::turn_cursor_off() { send_command(0x0C); }

void LcdFacade::move_cursor(LcdLine line, uint8_t offset)
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
