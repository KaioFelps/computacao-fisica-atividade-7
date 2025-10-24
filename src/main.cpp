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

  LcdFacade::initialize_lcd();

  // Torna os primeiros 7 pinos do registrador D saídas (são os segmentos do
  // led).
  DDRD |= 0b01111111;

  // Torna os 3 primeiros pinos do registrador B em saídas (são os pinos dos
  // leds).
  DDRB |= 0b00000111;

  // Torna os bits PC0 e PC1 do registrador DDRC inputs (são as entradas dos
  // potenciômetros), apesar de que já são entradas por padrão.
  DDRC &= ~(1 << PC0);
  DDRC &= ~(1 << PC1);

  // Configuração do conversor analógico-digital do microcontrolador do
  // Arduino (ATmega328).
  //
  // Esse trecho coloca o ADMUX num estado 01 (REF0:REF1) que diz que a tensão
  // usada pela porta AVCC (que alimenta o circuito analógico do
  // microcontrolador) será a mesma tensão usada como a tensão de referência
  // (valor máximo que o ADC pode medir).
  ADMUX &= ~((1 << REFS0) | (1 << REFS1));
  ADMUX |= (1 << REFS0);

  // * ADCSRx: ADC Control and Status Register x
  // * ADPSx: ADC Prescaler Select Bit x (é o x-nésimo bit menos significativo
  // do registrador ADCSRx)

  // Zera os bits do controlador & status B.
  // Esse registrador controla a fonte de gatilho, entrada e modo do ADC. Ao
  // zerar, estamos:
  // * desabilitando a inicialização automática do ADC
  // * deixando o ADC funcionar no seu modo simples de medição
  // * desligamos outras configurações avançadas do ADC
  ADCSRB = 0;

  // Configura individualmente os 3 primeiros bits do controlador & status A,
  // que definem o clock do conversor ADC (e outras funcionalidades do ADC).
  ADCSRA &= 0b11111000;
  // Essa configuração (0b111 = 128) divide o clock principal do
  // microcontrolador por 128 para formar o clock de trabalho do ADC, isso é,
  // declara que ADDCLK = CLK / 128.
  //
  // Para obter a precisão máxima de um ADC de 10 bits, o clock de conversão
  // deve estar entre 50kHz e 200kHz, e com esse cálculo, conseguimos chegar num
  // valor próximo, pois o arduino uno tem um clock de 16MHz, e 16000 (16 mega
  // hertz) / 128 = 125 (em kiloheartz).
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // ADIE: ADC Interrupt Enable
  // Permite que uma interrupção (`ADC_vect) seja acionada sempre que o ADC
  // terminar de medir a tensão e tiver um valor pronto.
  ADCSRA |= (1 << ADIE);

  // ADEN: ADC Enable
  // Liga o ADC manualmente (a nível de hardware, tirando essa parte do circuito
  // do modo de baixo consumo de energia).
  ADCSRA |= (1 << ADEN);

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
  // analógicos). Podemos desligar todos pois somente os 2 primeiros pinos estão
  // sendo utilizados, e são os do potenciômetro. Portanto, não afetamos nenhum
  // outro periférico. do pino em que o potenciômetro selecionado está conectado
  DIDR0 = 0b00111111;

  // Inicia a primeira conversão. As posteriores serão iniciadas ao final de
  // cada interrupção.
  ADCSRA |= (1 << ADSC);

  LcdFacade::send_command(0x8F);
  LcdFacade::send_character(0x25);
  LcdFacade::send_command(0xCF);
  LcdFacade::send_character(0x25);

  LcdFacade::send_command(0x80);
  LcdFacade::send_character(0);
  LcdFacade::send_command(0x80 + 0x40);
  LcdFacade::send_character(6);

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
