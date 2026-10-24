

// Copyright 2018-2026 Martin Ondrak
// This Example code was developed for https://github.com/dragon-engineer/esp32_fmdevkit
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Please note that FM broadcasting is only legal with a valid license or a very low power.
// The author is not responsible for any damages or penalties.

// You must use the correct Arduino Board!! ESP32 FM DevKit uses specific I2C and I2S pins

#include <Arduino.h>
#include <string.h>
#include <Ticker.h>
#include <Preferences.h>
#include "QN8027/src/QN8027.h"
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_SSD1306.h>
#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include "BluetoothA2DPSink.h"
#include "A2DPVolumeControl.h"
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/i2s_struct.h"
#include "symbols.h"


QN8027 fm;
Adafruit_SSD1306 oledDisplay(128, 64, &Wire); // 128x64
BluetoothA2DPSink a2dpSink;
A2DPLinearVolumeControl btVolume;

Ticker oneSecTick;
Ticker rdsTick;
Ticker rdsRefreshTick;

Preferences preferences;

float frequency = 87.5f;
float maybeFrequency = frequency; // When User changes the frequency with Switch/Button we use this temp variable
boolean frequencyChanged;

volatile boolean rdsTickFlag;
volatile boolean oledRefreshFlag = true;
volatile boolean rdsRefreshFlag;
volatile boolean oneSecFlag;
volatile boolean sw1Flag, sw2Flag, sw3Flag, sw4Flag;

volatile boolean btConnected;
volatile boolean btRdsReset;
volatile boolean btMetaPending;
volatile boolean mclkSyncPending;

int btAudioSamplingFreq = 44100; // Updated when the phone opens the A2DP stream

const char btName[] = "ESP32-FM_DevKit";
const char rdsName[] = "ESP32_FM";

char rdsTitle[RDSBUF_SIZE];
uint8_t rdsTitlePages;
uint8_t rdsDisplayPage;

boolean btLedBlinkState;

static const size_t kRdsField = 60;
char btTitle[kRdsField + 1];
char btArtist[kRdsField + 1];

// CS4344 single-speed MCLK/LRCK ratios are 256, 384, 512, 768 and 1024.
// The I2S master clock leaves through GPIO0. GPIO2 is the old MCLK pad and
// is left floating so a jumper from GPIO0 can feed the DAC.


void ICACHE_RAM_ATTR oneSec_callback(void) // One Second Ticker callback
{
	oneSecFlag = true;
}

void ICACHE_RAM_ATTR rdsTick_callback(void) // RDS Tick - we need this for paging the RDS Text if longer than 8 characters
{
	rdsTickFlag = true;
	// Frequency of this Ticker Event can be changed in symbols.h
}

void IRAM_ATTR rdsRefresh_callback(void) // RDS Refresh - We use this for streaming RDS info (every RDS page needs to be refreshed)
{
	rdsRefreshFlag = true;
	// Frequency of this Ticker Event can be changed in symbols.h
}

// CS4344 is Philips I2S: the sample MSB is one bit clock after LRCK.
// GPIO0 is CLK_OUT1, source 0, which is the I2S master clock. GPIO2 used to
// carry that net and must not drive it once the two pins are tied.
static void syncDacMclk(void)
{
	gpio_set_direction((gpio_num_t)I2S_MCLK, GPIO_MODE_INPUT);
	gpio_set_pull_mode((gpio_num_t)I2S_MCLK, GPIO_FLOATING);

	I2S0.conf.tx_msb_shift = 1;
	I2S0.conf.tx_right_first = 0;
	I2S0.conf.tx_short_sync = 0;

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
	REG_WRITE(PIN_CTRL, 0xFFF0);

	uint32_t ratio = (uint32_t)I2S0.sample_rate_conf.tx_bck_div_num *
		(uint32_t)I2S0.sample_rate_conf.tx_bits_mod * 2;
	Serial.printf("DAC MCLK on GPIO0: Fs %u, MCLK/LRCK %lu, APLL %u, Philips %u\r\n",
		btAudioSamplingFreq, ratio, I2S0.clkm_conf.clka_en, I2S0.conf.tx_msb_shift);
}

static void disableWifi(void)
{
	// Never start the station. If anything already brought the driver up, mode OFF stops it and releases the buffers.
	WiFi.persistent(false);
	WiFi.mode(WIFI_OFF);
	Serial.println("WiFi disabled");
}

// RDS radios only display a small ASCII set. Fold the letters that show up
// in Spanish, Czech, and other Latin metadata, and drop the rest.
static uint8_t rdsFold(uint32_t cp, char out[3])
{
	out[0] = out[1] = out[2] = 0;
	if (cp < 0x20 || cp == 0x7F)
	{
		out[0] = ' ';
		return 1;
	}
	if (cp < 0x80)
	{
		out[0] = (char)cp;
		return 1;
	}
	if (cp >= 0x0300 && cp <= 0x036F) return 0;

	if (cp == 0x00C6 || cp == 0x00E6)
	{
		out[0] = (cp == 0x00C6) ? 'A' : 'a';
		out[1] = (cp == 0x00C6) ? 'E' : 'e';
		return 2;
	}
	if (cp == 0x00DF)
	{
		out[0] = 's';
		out[1] = 's';
		return 2;
	}
	if (cp >= 0x00C0 && cp <= 0x00FF)
	{
		static const char latin1[] =
			"AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTs"
			"aaaaaaaceeeeiiiidnooooo/ouuuuyty";
		static_assert(sizeof(latin1) == 65, "Latin-1 fold table");
		out[0] = latin1[cp - 0x00C0];
		return 1;
	}
	if (cp == 0x2026)
	{
		out[0] = '.';
		out[1] = '.';
		out[2] = '.';
		return 3;
	}

	switch (cp)
	{
	case 0x00A0: out[0] = ' '; return 1;
	case 0x00A1: out[0] = '!'; return 1;
	case 0x00BF: out[0] = '?'; return 1;
	case 0x0104: out[0] = 'A'; return 1;
	case 0x0105: out[0] = 'a'; return 1;
	case 0x010C: out[0] = 'C'; return 1;
	case 0x010D: out[0] = 'c'; return 1;
	case 0x010E: out[0] = 'D'; return 1;
	case 0x010F: out[0] = 'd'; return 1;
	case 0x0110: out[0] = 'D'; return 1;
	case 0x0111: out[0] = 'd'; return 1;
	case 0x0118: out[0] = 'E'; return 1;
	case 0x0119: out[0] = 'e'; return 1;
	case 0x011A: out[0] = 'E'; return 1;
	case 0x011B: out[0] = 'e'; return 1;
	case 0x0139: out[0] = 'L'; return 1;
	case 0x013A: out[0] = 'l'; return 1;
	case 0x013D: out[0] = 'L'; return 1;
	case 0x013E: out[0] = 'l'; return 1;
	case 0x0141: out[0] = 'L'; return 1;
	case 0x0142: out[0] = 'l'; return 1;
	case 0x0143: out[0] = 'N'; return 1;
	case 0x0144: out[0] = 'n'; return 1;
	case 0x0147: out[0] = 'N'; return 1;
	case 0x0148: out[0] = 'n'; return 1;
	case 0x0150: out[0] = 'O'; return 1;
	case 0x0151: out[0] = 'o'; return 1;
	case 0x0152: out[0] = 'O'; out[1] = 'E'; return 2;
	case 0x0153: out[0] = 'o'; out[1] = 'e'; return 2;
	case 0x0154: out[0] = 'R'; return 1;
	case 0x0155: out[0] = 'r'; return 1;
	case 0x0158: out[0] = 'R'; return 1;
	case 0x0159: out[0] = 'r'; return 1;
	case 0x015A: out[0] = 'S'; return 1;
	case 0x015B: out[0] = 's'; return 1;
	case 0x0160: out[0] = 'S'; return 1;
	case 0x0161: out[0] = 's'; return 1;
	case 0x0164: out[0] = 'T'; return 1;
	case 0x0165: out[0] = 't'; return 1;
	case 0x016E: out[0] = 'U'; return 1;
	case 0x016F: out[0] = 'u'; return 1;
	case 0x0170: out[0] = 'U'; return 1;
	case 0x0171: out[0] = 'u'; return 1;
	case 0x0179: out[0] = 'Z'; return 1;
	case 0x017A: out[0] = 'z'; return 1;
	case 0x017B: out[0] = 'Z'; return 1;
	case 0x017C: out[0] = 'z'; return 1;
	case 0x017D: out[0] = 'Z'; return 1;
	case 0x017E: out[0] = 'z'; return 1;
	case 0x2010: case 0x2011: case 0x2012:
	case 0x2013: case 0x2014: case 0x2015:
	case 0x2212: out[0] = '-'; return 1;
	case 0x2018: case 0x2019: case 0x201A:
	case 0x201B: out[0] = '\''; return 1;
	case 0x201C: case 0x201D: case 0x201E:
	case 0x201F: out[0] = '"'; return 1;
	default: return 0;
	}
}

static void rdsSanitize(char *dest, size_t destSize, const char *src)
{
	size_t out = 0;
	bool pendingSpace = false;
	const uint8_t *p = (const uint8_t *)src;

	if (destSize == 0) return;
	dest[0] = 0;
	if (src == nullptr) return;

	while (*p != 0 && out + 1 < destSize)
	{
		uint32_t cp = 0;
		if (*p < 0x80)
		{
			cp = *p++;
		}
		else if ((*p & 0xE0) == 0xC0 && (p[1] & 0xC0) == 0x80)
		{
			cp = ((uint32_t)(p[0] & 0x1F) << 6) | (p[1] & 0x3F);
			p += 2;
		}
		else if ((*p & 0xF0) == 0xE0 && (p[1] & 0xC0) == 0x80 && (p[2] & 0xC0) == 0x80)
		{
			cp = ((uint32_t)(p[0] & 0x0F) << 12) | ((uint32_t)(p[1] & 0x3F) << 6) | (p[2] & 0x3F);
			p += 3;
		}
		else if ((*p & 0xF8) == 0xF0 && (p[1] & 0xC0) == 0x80 && (p[2] & 0xC0) == 0x80 && (p[3] & 0xC0) == 0x80)
		{
			p += 4;
			continue;
		}
		else
		{
			p++;
			continue;
		}

		char folded[3];
		uint8_t count = rdsFold(cp, folded);
		for (uint8_t i = 0; i < count && out + 1 < destSize; i++)
		{
			char ch = folded[i];
			if (ch == ' ')
			{
				pendingSpace = (out > 0);
				continue;
			}
			if (pendingSpace)
			{
				if (out + 2 >= destSize) break;
				dest[out++] = ' ';
				pendingSpace = false;
			}
			dest[out++] = ch;
		}
	}
	dest[out] = 0;
}

static void rdsCompose(char *out, size_t outSize, const char *title, const char *artist)
{
	bool hasTitle = title != nullptr && title[0] != 0;
	bool hasArtist = artist != nullptr && artist[0] != 0;
	if (outSize == 0) return;
	if (hasTitle && hasArtist) snprintf(out, outSize, "%s - %s", title, artist);
	else if (hasTitle) snprintf(out, outSize, "%s", title);
	else if (hasArtist) snprintf(out, outSize, "%s", artist);
	else out[0] = 0;
}

static void rdsCopyPage(char out[9])
{
	memset(out, ' ', 8);
	out[8] = 0;

	const char *src = rdsName;
	size_t start = 0;
	if (rdsDisplayPage > 0 && rdsTitle[0] != 0)
	{
		src = rdsTitle;
		start = (size_t)(rdsDisplayPage - 1) * 8;
	}

	size_t length = strlen(src);
	if (start >= length) return;
	size_t count = length - start;
	if (count > 8) count = 8;
	memcpy(out, src + start, count);
}

static void rdsPublishPage(void)
{
	char page[9];
	rdsCopyPage(page);
	fm.setProgramService(page);
}

static void rdsUseStation(void)
{
	rdsTitle[0] = 0;
	rdsTitlePages = 0;
	rdsDisplayPage = 0;
	fm.setRadiotext(rdsName);
	rdsPublishPage();
}

static void rdsUseTitle(const char *title)
{
	if (strcmp(rdsTitle, title) == 0) return;

	strncpy(rdsTitle, title, RDSBUF_SIZE - 1);
	rdsTitle[RDSBUF_SIZE - 1] = 0;
	size_t length = strlen(rdsTitle);
	rdsTitlePages = (uint8_t)((length + 7) / 8);
	if (rdsTitlePages == 0) rdsTitlePages = 1;
	rdsDisplayPage = 1;
	fm.setRadiotext(rdsTitle);
	rdsPublishPage();
}

static void rdsAdvancePage(void)
{
	if (rdsTitlePages == 0)
	{
		rdsDisplayPage = 0;
		return;
	}

	if (rdsDisplayPage >= rdsTitlePages) rdsDisplayPage = 0;
	else rdsDisplayPage++;
	rdsPublishPage();
}

// Fired from the main loop - refresh the OLED
void oledRefresh(void)
{
	oledDisplay.fillRect(0, 0, 128, 64, BLACK);

	oledDisplay.setCursor(3, 3);
	oledDisplay.setTextSize(1);
	char rds[9];
	rdsCopyPage(rds);
	oledDisplay.printf("RDS: %c%c%c%c%c%c%c%c", rds[0], rds[1], rds[2], rds[3], rds[4], rds[5], rds[6], rds[7]);

	if (frequencyChanged)
	{
		oledDisplay.setCursor(120, 3);
		oledDisplay.printf("!");
	}

	oledDisplay.setCursor(3, 26);
	oledDisplay.setTextSize(2);
	oledDisplay.printf("%3.02f MHz", maybeFrequency);

	oledDisplay.setTextSize(1);
	oledDisplay.setCursor(3, 52);
	if (btConnected)
	{
		char peer[19];
		strncpy(peer, a2dpSink.get_peer_name(), sizeof(peer) - 1);
		peer[sizeof(peer) - 1] = 0;
		oledDisplay.printf("BT %.18s", peer);
	}
	else
	{
		oledDisplay.print("BT waiting");
	}

	oledDisplay.display();
}

// Switch/Button Callbacks - ESC Button
void IRAM_ATTR sw1_callback(void)
{
	sw1Flag = true;
}

// Switch/Button Callbacks - OK Button
void IRAM_ATTR sw2_callback(void)
{
	sw2Flag = true;
}

// Switch/Button Callbacks - DOWN Button
void IRAM_ATTR sw3_callback(void)
{
	sw3Flag = true;
}

// Switch/Button Callbacks - UP Button
void IRAM_ATTR sw4_callback(void)
{
	sw4Flag = true;
}

static void btConnectionChanged(esp_a2d_connection_state_t state, void *)
{
	btConnected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
	if (!btConnected) btRdsReset = true;
	oledRefreshFlag = true;
}

static void btSampleRateChanged(uint16_t rate)
{
	btAudioSamplingFreq = rate;
	// apply_audio_info() reprograms I2S0 immediately after this returns.
	mclkSyncPending = true;
}

static void btMetadata(uint8_t id, const uint8_t *text)
{
	char *dest = nullptr;
	if (id == ESP_AVRC_MD_ATTR_TITLE) dest = btTitle;
	else if (id == ESP_AVRC_MD_ATTR_ARTIST) dest = btArtist;
	if (dest == nullptr || text == nullptr || text[0] == 0) return;

	char cleaned[kRdsField + 1];
	rdsSanitize(cleaned, sizeof(cleaned), (const char *)text);
	if (cleaned[0] == 0) return;
	strncpy(dest, cleaned, kRdsField);
	dest[kRdsField] = 0;
	btMetaPending = true;
	oledRefreshFlag = true;
}

void setup(void)
{
	Serial.begin(115200);

	Serial.printf("\r\nESP32 Chip Revision: %u\r\n\r\n", ESP.getChipRevision());
	// Revision 0 had a bug in audio PLL design but all the chips now should be Rev 1

	disableWifi();

	preferences.begin("fmdevkit"); // Store user frequency in FLASH
	frequency = preferences.getFloat("frequency", FREQ_DEFAULT);
	maybeFrequency = frequency;

	pinMode(LED_BUILTIN, OUTPUT); // Blue built-in LED
	digitalWrite(LED_BUILTIN, HIGH);
	btLedBlinkState = true;

	pinMode(SW1, INPUT);
	pinMode(SW2, INPUT);
	pinMode(SW3, INPUT);
	pinMode(SW4, INPUT);

	attachInterrupt((SW1), sw1_callback, FALLING); // Register all input callback
	attachInterrupt((SW2), sw2_callback, FALLING);
	attachInterrupt((SW3), sw3_callback, FALLING);
	attachInterrupt((SW4), sw4_callback, FALLING);


	// QN8027. These are the values that are on air: 0 dB in, 75 kHz deviation, 50 µs.
	fm.setCrystal(QN8027::Crystal::Mhz12);
	fm.setClockSource(QN8027::ClockSource::Crystal);
	fm.setCrystalDrive(0x0F);
	fm.setInputGain(QN8027::InputGain::Db6);
	fm.setDigitalGain(QN8027::DigitalGain::Db3);
	fm.setInputResistance(QN8027::InputResistance::Ohm20k);
	fm.setPreEmphasis(QN8027::PreEmphasis::Us50);
	fm.setPowerSave(QN8027::PowerSave::Disabled);
	fm.setPilotDeviation(QN8027::PilotCode);
	fm.setDeviationKhz(75);
	fm.setStereo(true);
	fm.mute(false);
	fm.setTxPower(75);
	fm.setRdsPi(0x4000);
	fm.setProgramType(QN8027::ProgramType::Pop);
	fm.setRdsDeviationKhz(3.5f);
	fm.setRdsEnabled(RDS_ENABLED);
	fm.begin(SDA, SCL, 100000, frequency);
	rdsUseStation();


	//Demo screen
	oledDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false);
	oledDisplay.setTextColor(WHITE);

	oledDisplay.clearDisplay();
	oledDisplay.display();


	oledDisplay.setTextSize(1);
	oledDisplay.setCursor(0, 0);
	oledDisplay.printf("FM DevKit");

	oledDisplay.setTextSize(3);
	oledDisplay.setCursor(0, 16);
	oledDisplay.printf("ESP32");

	oledDisplay.setTextSize(1);
	oledDisplay.setCursor(0, 48);
	oledDisplay.printf("BT sink %s", btName);

	oledDisplay.display();

	i2s_config_t i2sConfig = {
		.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
		.sample_rate = 44100,
		.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format = I2S_COMM_FORMAT_STAND_I2S,
		.intr_alloc_flags = 0,
		.dma_buf_count = 8,
		.dma_buf_len = 128,
		.use_apll = true,
		.tx_desc_auto_clear = true,
		.fixed_mclk = 0,
		.mclk_multiple = I2S_MCLK_MULTIPLE_256,
		.bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
	};
	i2s_pin_config_t i2sPins = {
		.mck_io_num = 0,
		.bck_io_num = I2S_SCLK,
		.ws_io_num = I2S_LRCLK,
		.data_out_num = I2S_DOUT,
		.data_in_num = I2S_PIN_NO_CHANGE
	};
	a2dpSink.set_i2s_config(i2sConfig);
	a2dpSink.set_pin_config(i2sPins);
	a2dpSink.set_volume_control(&btVolume);
	a2dpSink.set_auto_reconnect(true);
	a2dpSink.set_on_connection_state_changed(btConnectionChanged);
	a2dpSink.set_sample_rate_callback(btSampleRateChanged);
	a2dpSink.set_avrc_metadata_callback(btMetadata);
	a2dpSink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);

	Serial.printf("Free heap before Bluetooth: %lu bytes\r\n", ESP.getFreeHeap());
	a2dpSink.start(btName);
	syncDacMclk();
	Serial.printf("Bluetooth sink \"%s\" is discoverable\r\n", btName);

	oneSecTick.attach_ms(1000, oneSec_callback);
	rdsTick.attach_ms(RDS_PAGING_INTERVAL, rdsTick_callback);
	rdsRefreshTick.attach_ms(RDS_REFRESH_INTERVAL, rdsRefresh_callback);
}


void loop(void)
{
	if (mclkSyncPending)
	{
		mclkSyncPending = false;
		syncDacMclk();
	}

	if (btRdsReset)
	{
		btRdsReset = false;
		btMetaPending = false;
		btTitle[0] = 0;
		btArtist[0] = 0;
		rdsUseStation();
		oledRefreshFlag = true;
	}

	if (btMetaPending)
	{
		btMetaPending = false;
		char title[kRdsField + 1];
		char artist[kRdsField + 1];
		char line[RDSBUF_SIZE];
		strncpy(title, btTitle, kRdsField);
		title[kRdsField] = 0;
		strncpy(artist, btArtist, kRdsField);
		artist[kRdsField] = 0;
		rdsCompose(line, sizeof(line), title, artist);
		if (line[0] != 0) rdsUseTitle(line);
		oledRefreshFlag = true;
	}

	if (oneSecFlag)
	{
		if (btConnected) btLedBlinkState = !btLedBlinkState;
		else btLedBlinkState = true;

		Serial.printf("BT %s, FS %u Hz, peer %s\r\n",
			btConnected ? "connected" : "waiting",
			btAudioSamplingFreq,
			a2dpSink.get_peer_name());

		oneSecFlag = false;
	}


	if (rdsRefreshFlag)
	{
		fm.rdsRefresh();

		rdsRefreshFlag= false;
	}

	if (rdsTickFlag)
	{

		rdsAdvancePage();
		oledRefreshFlag = true;

		rdsTickFlag = false;
	}

	// Switch/Buttons handlers
	if (sw1Flag)
	{
		//ESC
		Serial.println("ESC");

		if (frequencyChanged)
		{
			maybeFrequency = frequency;
			frequencyChanged = false;

			oledRefreshFlag = true;
		}

		sw1Flag = false;
	}
	else if (sw2Flag)
	{
		//OK
		Serial.println("OK");

		frequency = maybeFrequency;
		fm.setFrequency(frequency);
		preferences.putFloat("frequency", frequency);
		frequencyChanged = false;

		oledRefreshFlag = true;

		sw2Flag = false;
	}
	else if (sw3Flag)
	{
		//DOWN
		Serial.println("DOWN");

		if (maybeFrequency > 87.5f) maybeFrequency -= 0.1f;
		else maybeFrequency = 108.0f;

		frequencyChanged = true;

		oledRefreshFlag = true;

		sw3Flag = false;


	}
	else if (sw4Flag)
	{
		//UP
		Serial.println("UP");

		if (maybeFrequency < 108.0f) maybeFrequency = maybeFrequency + 0.1f;
		else maybeFrequency = 87.5f;

		frequencyChanged = true;

		Serial.printf("Freq: %3.2f\r\n", maybeFrequency);

		oledRefreshFlag = true;

		sw4Flag = false;
	}


	if (oledRefreshFlag)
	{

		oledRefresh();

		oledRefreshFlag = false;
	}



	digitalWrite(LED_BUILTIN, btLedBlinkState);

}
