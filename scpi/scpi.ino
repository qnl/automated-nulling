#include <SPI.h>
#include <EEPROM.h>
#include "Parser.h"

/******************************************************************************/
/* Defines                                                                    */
/******************************************************************************/

#define BAUDRATE        9600
#define MAXBUFLEN       128

#define VERSIONINFO     "Quantum Nanoelectronics Laboratory, Multi-LTC2704, V1.1"

#define SS_PIN          17
#define SCK_PIN         13
#define MOSI_PIN        11
#define MISO_PIN        12
#define SSA0            4
#define SSA1            15
#define SSA2            16

#define NUM_CHANNELS    24
#define NUM_CODEBITS    16

#define NUM_ADDRS       4
#define ADDR_A          0b0000
#define ADDR_B          0b0010
#define ADDR_C          0b0100
#define ADDR_D          0b0110

#define NUM_SPANS       6
#define SPAN_0_2        0b0000
#define SPAN_0_4        0b0001
#define SPAN_N2_2       0b0010
#define SPAN_N4_4       0b0011
#define SPAN_N1_1       0b0100
#define SPAN_N1_3       0b0101

#define CTRL_W_VAL      0b1001
#define CTRL_R_VAL      0b1101
#define CTRL_W_SPAN     0b1000
#define CTRL_R_SPAN     0b1100

SCPI_Parser parser;

struct channel_t {
  uint16_t data;
  uint16_t span;
};

uint8_t span_codes[NUM_SPANS] = {
  SPAN_0_2, SPAN_0_4, SPAN_N2_2, SPAN_N4_4, SPAN_N1_1, SPAN_N1_3
};
uint8_t addr_codes[NUM_ADDRS] = {
  ADDR_A, ADDR_B, ADDR_C, ADDR_D
};
uint8_t chip_codes[NUM_CHANNELS / 4] = {0, 1, 2, 3, 4, 5};

channel_t slowdac[NUM_CHANNELS];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);
  while (!Serial);
  initialize();

  parser.RegisterCommand(F("*IDN?"), &handleIdentify);
  parser.RegisterCommand(F("*PRINT"), &handlePrint);
  parser.RegisterCommand(F("*RST"), &handleReset);

  parser.RegisterCommand(F("CH#:SPAN"), &handleSetSpan);
  parser.RegisterCommand(F("CH#:SPAN?"), &handleGetSpan);
  parser.RegisterCommand(F("CH#:CODE"), &handleSetCode);
  parser.RegisterCommand(F("CH#:CODE?"), &handleGetCode);

  parser.RegisterCommand(F("PIN#"), &handleSetPin);
  parser.RegisterCommand(F("PIN#?"), &handleGetPin);
  
  parser.RegisterCommand(F(":EEPROM:SAVE"), &handleEESave);
  parser.RegisterCommand(F(":EEPROM:REST"), &handleEERestore);
  parser.RegisterCommand(F(":EEPROM:VALID?"), &handleEEValid);
  parser.SetErrorHandler(&handleError);
}

void loop() {
  // put your main code here, to run repeatedly:
  parser.ProcessInput(Serial, "\n");
}

/******************************************************************************/
/* Handlers                                                                   */
/******************************************************************************/

void handleIdentify(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  Serial.println(VERSIONINFO);
}

void handlePrint(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  printSlowDAC();
}

void handleReset(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int error = 0;
  
  if (parameters.First() == NULL) {
    error = restoreChannel(NUM_CHANNELS);
  } else if (isInteger(parameters.First())) {
    error = restoreChannel(atoi(parameters.First()));
  } else {
    error = 1;
  }

  if (error) handleError(commands, parameters, interface);
}

void handleSetPin(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int pin, state;

  if ((sscanf(commands.First(), "PIN%d", &pin) == EOF) || 
      (sscanf(parameters.First(), "%d", &state) == EOF)) {
    handleError(commands, parameters, interface);
    return;
  } else if (!(pin == SS_PIN || pin == SSA0 || pin == SSA1 || pin == SSA2)) {
    handleError(commands, parameters, interface);
    return;
  }

  digitalWrite(pin, state ? HIGH : LOW);
}

void handleGetPin(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int pin, state;

  if (sscanf(commands.First(), "PIN%d", &pin) == EOF) {
    handleError(commands, parameters, interface);
    return;
  } else if (!(pin == SS_PIN || pin == SSA0 || pin == SSA1 || pin == SSA2)) {
    handleError(commands, parameters, interface);
    return;
  }

  state = digitalRead(pin);

  Serial.println(state);
}

void handleSetSpan(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int channel, span;

  if ((sscanf(commands.First(), "CH%d", &channel) == EOF) || 
      (sscanf(parameters.First(), "%d", &span) == EOF)) {
    handleError(commands, parameters, interface);
    return;
  } else if (channel < 0 || channel >= NUM_CHANNELS || span < 0 || span >= NUM_SPANS) {
    handleError(commands, parameters, interface);
    return;
  }

  uint16_t sc = span_codes[span];
  transferSequence(channel, CTRL_W_SPAN, sc);
  slowdac[channel].span = (uint16_t)span;
}

void handleGetSpan(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int channel;
  
  if (sscanf(commands.First(), "CH%d", &channel) == EOF) {
    handleError(commands, parameters, interface);
    return;
  } else if (channel < 0 || channel >= NUM_CHANNELS) {
    handleError(commands, parameters, interface);
    return;
  }

  Serial.println(slowdac[channel].span);
}

void handleSetCode(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int channel;
  uint32_t code = 0;
  
  if ((sscanf(commands.First(), "CH%d", &channel) == EOF) || 
      (sscanf(parameters.First(), "%u", &code) == EOF)) {
    handleError(commands, parameters, interface);
    return;
  } else if (channel < 0 || channel >= NUM_CHANNELS || code < 0 || code >= (1L << NUM_CODEBITS)) {
    Serial.println(code);
    handleError(commands, parameters, interface);
    return;
  }

  transferSequence(channel, CTRL_W_VAL, (uint16_t) code);
  slowdac[channel].data = code;
}

void handleGetCode(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int channel;

  if (sscanf(commands.First(), "CH%d", &channel) == EOF) {
    handleError(commands, parameters, interface);
    return;
  } else if (channel < 0 || channel >= NUM_CHANNELS) {
    handleError(commands, parameters, interface);
    return;
  }

  Serial.println(slowdac[channel].data);
}

void handleEESave(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int error = 0;
  
  if (parameters.First() == NULL) {
    error = saveChannel(NUM_CHANNELS);
  } else if (isInteger(parameters.First())) {
    error = saveChannel(atoi(parameters.First()));
  } else {
    error = 1;
  }

  if (error) handleError(commands, parameters, interface);
}

void handleEERestore(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  int error = 0;
  
  if (parameters.First() == NULL) {
    error = restoreChannel(NUM_CHANNELS);
  } else if (isInteger(parameters.First())) {
    error = restoreChannel(atoi(parameters.First()));
  } else {
    error = 1;
  }

  if (error) handleError(commands, parameters, interface);
}

void handleEEValid(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  Serial.println(validateEEPROM()?"TRUE":"FALSE");
}

void handleError(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  char msg[100];

  int num_cmds = commands.Size();
  int num_params = parameters.Size();
  
  snprintf(msg, sizeof(msg), "Command size: %d, Parameter size: %d", num_cmds, num_params);
  Serial.print("ERROR ");
  Serial.println(msg);

  for (int i = 0; i < num_cmds; i++) {
    if (i == 0) Serial.print("Commands: ");
    Serial.print(commands.Get(i));

    if (i != num_cmds- 1) {
      Serial.print(", ");
    } else {
      Serial.println("");
    }
  }

  for (int i = 0; i < num_params; i++) {
    if (i == 0) Serial.print("Parameters: ");
    Serial.print(parameters.Get(i));

    if (i != num_params - 1) {
      Serial.print(", ");
    } else {
      Serial.println("");
    }
  }
}

int getCommandChannel(char* chstr) {
  int channel;
  
  if (sscanf(chstr, "CH%d", &channel) == EOF) return -1;

  return channel;
}

int getIntParameter(char* str) {
  int param;

  if (sscanf(str, "%d", &param) == EOF) return -1;

  return param;
}

bool isInteger(char* str) {
  if (strlen(str) == 0) return false;
  
  bool out = true;
  for (int i = 0; i < strlen(str); i++) {
    out = out && isDigit(str[i]);
  }

  return out;
}

/******************************************************************************/
/* DAC Utils                                                                  */
/******************************************************************************/

void initialize() { 
  // Setting pin modes
  pinMode(SS_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, OUTPUT);
  pinMode(SSA0, OUTPUT);
  pinMode(SSA1, OUTPUT);
  pinMode(SSA2, OUTPUT);

  digitalWrite(SS_PIN, HIGH);
  digitalWrite(SSA0, LOW);
  digitalWrite(SSA1, LOW);
  digitalWrite(SSA2, LOW);

  delay(100);

  if (validateEEPROM()) {
    restoreChannel(NUM_CHANNELS);
  } else {
//    Serial.println("Corrupted EEPROM Memory");
  }
}

void printSlowDAC() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print("(SPAN, DATA) = ");
    Serial.print(slowdac[i].span);
    Serial.print(", ");
    Serial.println(slowdac[i].data);
  }
}

void printData(byte data[3]) {
  for (int b = 0; b < 3; b++) {
    byte current = data[b];
    for (int i = 0; i < 8; i++) {
      bool val = current & 0x80;
      Serial.print(val);
      current = current << 1;
    }
  }
  Serial.println("");
}

int getChip(int channel) {
  return chip_codes[channel / 4];
}

int getAddress(int channel) {
  return addr_codes[channel % 4];
}

/******************************************************************************/
/* EEPROM Utils                                                               */
/******************************************************************************/

int saveChannel(int channel) {
  if (channel > NUM_CHANNELS || channel < 0) return 1;

  if (channel == NUM_CHANNELS) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      EEPROM.put(i * sizeof(channel_t), slowdac[i]);
    }
  } else {
    EEPROM.put(channel * sizeof(channel_t), slowdac[channel]);
  }

  uint32_t checksum = eeprom_crc();
  EEPROM.put(sizeof(channel_t) * NUM_CHANNELS, checksum);

  return 0;
}

int restoreChannel(int channel) {
  if (channel > NUM_CHANNELS || channel < 0) return 1;
  
  channel_t chan_data;

  if (channel == NUM_CHANNELS) {
    int error = 0;
    for (int i = 0; i < NUM_CHANNELS; i++ ) {
      EEPROM.get(i * sizeof(channel_t), chan_data);
      if (!validateSpan(&chan_data)) {
        error = 2;
        continue;
      }
      
      slowdac[i] = chan_data;
      transferSequence(i, CTRL_W_VAL, slowdac[i].data);
      transferSequence(i, CTRL_W_SPAN, slowdac[i].span);
    }
    return error;
  } else {
    EEPROM.get(channel * sizeof(channel_t), chan_data);
    if (!validateSpan(&chan_data)) return 2;
    
    slowdac[channel] = chan_data;
    transferSequence(channel, CTRL_W_VAL, slowdac[channel].data);
    transferSequence(channel, CTRL_W_SPAN, slowdac[channel].span);
    return 0;
  }
}

int validateSpan(channel_t* channel) {
  return (channel->span >= 0 && channel->span < NUM_SPANS);
}

uint32_t eeprom_crc(void) {
  /* https://docs.arduino.cc/learn/programming/eeprom-guide#eeprom-crc  */
  const uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  uint32_t crc = ~0L;

  for (int index = 0 ; index < sizeof(channel_t) * NUM_CHANNELS; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }

  return crc;
}

int validateEEPROM(void) {
  uint32_t checksum = eeprom_crc();
  uint32_t stored;

  EEPROM.get(NUM_CHANNELS * sizeof(channel_t), stored);

  return checksum == stored;
}

/******************************************************************************/
/* SPI Utils                                                                  */
/******************************************************************************/

void selectChip(int chip) {
  digitalWrite(SSA0, bitRead(chip, 0) ? HIGH : LOW);
  digitalWrite(SSA1, bitRead(chip, 1) ? HIGH : LOW);
  digitalWrite(SSA2, bitRead(chip, 2) ? HIGH : LOW);
  delay(5);
}

uint16_t transferSequence(int channel, byte ctrl, uint16_t data) {
  byte addr = getAddress(channel);
  byte sequence[3];

  selectChip(getChip(channel));

  sequence[0] = (ctrl << 4 & 0xF0) | (addr & 0x0F);
  sequence[1] = (byte)((data >> 8) & 0xff);
  sequence[2] = (byte)(data & 0xff);
   
  spiTransferBytes(sequence, sizeof(sequence));
  delay(5);

  return (uint16_t) ((sequence[1] << 8 & 0xff00) | (sequence[2] & 0x00ff));
}

void spiTransferBytes(byte* data, int num_bytes) {
  bool bitval;
  
  digitalWrite(SS_PIN, LOW);
  for (int b = 0; b < num_bytes; b++) {
    for (int i = 7; i >= 0; i--) {
      bitval = spiWriteOneBit(bitRead(data[b], i));
//      Serial.print(bitval);
    }
  }
  digitalWrite(SS_PIN, HIGH);
//  Serial.println();
}

bool spiWriteOneBit(bool bitval) {
  bool outbit;
  digitalWrite(MOSI_PIN, bitval ? HIGH : LOW);
  digitalWrite(SCK_PIN, HIGH);
  digitalWrite(MOSI_PIN, bitval ? HIGH : LOW);
  digitalWrite(SCK_PIN, LOW);
  outbit = digitalRead(MISO_PIN);
  return outbit;
}
