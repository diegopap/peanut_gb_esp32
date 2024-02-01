#include "Arduino.h"

// Peanut-GB emulator settings
#define ENABLE_SOUND 1
#define ENABLE_LCD 1
#define PEANUT_GB_HIGH_LCD_ACCURACY 1
#define PEANUT_GB_USE_BIOS 0

#include "sound.h"
#include "peanut_gb.c"
// #include "rom.h"

// Touch inputs for buttons
#define PIN_UP     T4
#define PIN_DOWN   T5
#define PIN_LEFT   T6
#define PIN_RIGHT  T7
#define PIN_A      T9
#define PIN_B      T3
#define PIN_START  T2
#define PIN_SELECT T0
#define TOUCH_THRESHOLD 40

// GPIO connections for TFT
#define TFT_CS   GPIO_NUM_16
#define TFT_DC   GPIO_NUM_5
#define TFT_RST  GPIO_NUM_19  
#define TFT_MOSI GPIO_NUM_23 
#define TFT_SCLK GPIO_NUM_18 

// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 17;
//
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 34;
const uint8_t SOFT_MOSI_PIN = 22;
const uint8_t SOFT_SCK_PIN = 21;

// SdFat software SPI template
#include "SdFat.h"
#include "sdios.h"
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
// Speed argument is ignored for software SPI.
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50), &softSpi)
SdFat SD;
SdFile dirFile;
SdFile file;

// Global variables
static struct gb_s gb;       // Emulator context. Only values within the `direct` struct may be modified directly by the front-end implementation.
static uint8_t ram[32768];   // Cartridge RAM

//Sound variables
#define SOUND_PIN 25
hw_timer_t *timer = NULL;

#include "soc/ledc_struct.h"

void readInputs()
{
    gb.direct.joypad_bits.up=touchRead(PIN_UP) < TOUCH_THRESHOLD ? 0 : 1;
    gb.direct.joypad_bits.down=touchRead(PIN_DOWN) < TOUCH_THRESHOLD ? 0 : 1;
    gb.direct.joypad_bits.left=touchRead(PIN_LEFT) < TOUCH_THRESHOLD ? 0 : 1;
    gb.direct.joypad_bits.right=touchRead(PIN_RIGHT) < TOUCH_THRESHOLD ? 0 : 1;
    gb.direct.joypad_bits.a=touchRead(PIN_A) < TOUCH_THRESHOLD ? 0 : 1;
    gb.direct.joypad_bits.b=touchRead(PIN_B) < TOUCH_THRESHOLD ? 0 : 1;
    gb.direct.joypad_bits.select=touchRead(PIN_SELECT) < TOUCH_THRESHOLD ? 0 : 1;
    gb.direct.joypad_bits.start=touchRead(PIN_START) < TOUCH_THRESHOLD ? 0 : 1;
}

void IRAM_ATTR sound_ISR(){
    auto& reg = LEDC.channel_group[0].channel[0];
    reg.duty.duty = audio_update() << 4; // 25 bit (21.4)
    reg.conf0.sig_out_en = 1; // This is the output enable control bit for channel
    reg.conf1.duty_start = 1; // When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware
    reg.conf0.clk_en = 1;

    readInputs();
  
}

#define TFT_HEIGHT 240
#define TFT_WIDTH 320
uint16_t video_buffer_scaled[TFT_WIDTH];

#define LGFX_USE_V1
#include "LovyanGFX.hpp"
lgfx::Panel_ILI9341 _panel_instance;
lgfx::Bus_SPI _bus_instance; // SPI

class LGFX: public lgfx::LGFX_Device {
public:
    LGFX(void) {
    {
        auto cfg = _bus_instance.config();
        // SPI
        cfg.spi_host = VSPI_HOST;
        cfg.spi_mode = 3;
        cfg.freq_write = 80000000; 
        cfg.freq_read = 80000000;
        cfg.spi_3wire = false;
        cfg.use_lock = false;
        cfg.dma_channel = 2;
        cfg.pin_sclk = TFT_SCLK;
        cfg.pin_mosi = TFT_MOSI;
        cfg.pin_miso = -1;
        cfg.pin_dc = TFT_DC;
        _bus_instance.config(cfg);
        _panel_instance.setBus( &_bus_instance);
    }
    }
};

static LGFX tft;

/**
 * @brief function called by Peanut-GB to draw a single line on the LCD
 * 
 * @param gb      emulator context
 * @param pixels  The 160 pixels to draw.
 * 			      Bits 1-0 are the colour to draw.
 * 		          Bits 5-4 are the palette, where:
 * 				  OBJ0 = 0b00,
 * 				  OBJ1 = 0b01,
 * 				  BG = 0b10
 * 			      Other bits are undefined.
 * 			      Bits 5-4 are only required by front-ends
 * 			      which want to use a different colour for
 * 			      different object palettes. This is what
 * 			      the Game Boy Color (CGB) does to DMG
 * 			      games.
 * @param line    Line to draw pixels on. This is
 *                guaranteed to be between 0-143 inclusive.
 */
void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
        const uint_fast8_t line)
{

    // Example of Game Boy Color palette
    const uint16_t palette[3][4] = {
        { 0xFFFF, 0x1F65, 0x1F00, 0x0000 },     // OBJ0
        { 0xFFFF, 0x10FC, 0xC789, 0x0000 },     // OBJ1
        { 0xFFFF, 0x1F65, 0x1F00, 0x0000 }      // BG
    };

    int index = 0;
    for(uint16_t x=0;x<LCD_WIDTH;x++) 
    {
        video_buffer_scaled[index++] = video_buffer_scaled[index++] = palette[(pixels[x] & LCD_PALETTE_ALL) >> 4]
                [pixels[x] & 3];
    }

    int lineScaled = line*5/3;
    if (line%3 == 0) {
        tft.pushImage(0, lineScaled, TFT_WIDTH, 1, (uint16_t *)(&video_buffer_scaled[0]), true);
    } else {
        tft.pushImage(0, lineScaled, TFT_WIDTH, 1, (uint16_t *)(&video_buffer_scaled[0]), true);
        tft.pushImage(0, lineScaled + 1, TFT_WIDTH, 1, (uint16_t *)(&video_buffer_scaled[0]), true);
    }
}

void video_init() {
    printf("Initializing ST7789 TFT...");
    auto cfg = _panel_instance.config();
    cfg.pin_cs = TFT_CS;
    cfg.pin_rst = TFT_RST;
    cfg.pin_busy = -1;
    cfg.memory_width = TFT_WIDTH;
    cfg.memory_height = TFT_HEIGHT;
    cfg.panel_width = TFT_WIDTH;
    cfg.panel_height = TFT_HEIGHT;
    cfg.offset_x = 0;
    cfg.offset_y = 0;
    cfg.offset_rotation = 4;
    cfg.dummy_read_pixel = 8;
    cfg.dummy_read_bits = 1;
    cfg.readable = false;
    cfg.invert = false;
    cfg.rgb_order = true;
    cfg.dlen_16bit = false;
    cfg.bus_shared = false;
    _panel_instance.config(cfg);
    tft.setPanel(&_panel_instance);
    tft.init();
    tft.initDMA();
    tft.fillScreen(0x0000);

    printf("[TFT initialization OK ]\n");
}

const void *ROM;

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
    return ((unsigned char *) ROM)[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
	return ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	ram[addr] = val;
}

/**
 * Notify front-end of error.
 */
void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t val)
{
    const char* gb_err_str[4] = {
			"UNKNOWN",
			"INVALID OPCODE",
			"INVALID READ",
			"INVALID WRITE"
		};
	printf("Error %d occurred: %s\n. Abort.\n",
		gb_err,
		gb_err >= GB_INVALID_MAX ?
		gb_err_str[0] : gb_err_str[gb_err]);
}

#define SAMPLING_RATE 32000

void attachInterruptTask(void *pvParameters) {
    timer = timerBegin(1, 80, true);
    timerAttachInterrupt(timer, &sound_ISR, true);
    timerAlarmWrite(timer, 1000000 / SAMPLING_RATE, true);
    timerAlarmEnable(timer);
    vTaskDelete(NULL);
}

void sound_init() {
    pinMode(SOUND_PIN, OUTPUT);

    ledcSetup(0,SAMPLING_RATE,8);
    ledcAttachPin(SOUND_PIN, 0);

    //Move sound to other core to improve performance. 
    xTaskCreatePinnedToCore(attachInterruptTask, "Attach Interrupt Task", 2000, NULL, 6, NULL, 0);

}

void clear_screen() {
    tft.clear();
    tft.setCursor(0,0);
}

char ROM_PATH[80] = {0};

uint8_t flashdata[4096] = {0}; //4kB buffer

#define MAX_FILE_SIZE 2 * 1024 * 1024

#define SPI_FLASH_ADDRESS 0x00200000

spi_flash_mmap_handle_t handle1;

void flash_rom() {
    clear_screen();
    tft.println("Loading ROM ...");
    if (!file.open(ROM_PATH, O_READ)) {
        printf("File NOT FOUND: ");
        printf(ROM_PATH);
    }
    uint64_t i = 0;
    ESP_ERROR_CHECK(spi_flash_erase_range(SPI_FLASH_ADDRESS, MAX_FILE_SIZE));
    while (file.available()) {
        file.read(flashdata, sizeof(flashdata));
        ESP_ERROR_CHECK(spi_flash_write(SPI_FLASH_ADDRESS + i*sizeof(flashdata), flashdata, sizeof(flashdata)));
        i++;
    } 
    ESP_ERROR_CHECK(spi_flash_mmap(SPI_FLASH_ADDRESS, MAX_FILE_SIZE, SPI_FLASH_MMAP_DATA, &ROM, &handle1));
}

#define MAXFILES 30
#define MAXFILENAME_LENGTH 80
char PATH[MAXFILENAME_LENGTH] = {0};
char filename[MAXFILENAME_LENGTH] = {0};
char first[MAXFILENAME_LENGTH] = {0};

int fileIndex = 0;

void show_files() {

    clear_screen();

    if (!dirFile.open(PATH, O_READ)) {
        printf("PATH NOT FOUND: ");
        printf(PATH);
    }

    boolean num = true;

    for (int i = 0; i < fileIndex; i++)
    {
        file.openNext(&dirFile, O_READ);
    }

    for (int i = fileIndex; i < fileIndex + MAXFILES; i++)
    {
        if (num && file.openNext(&dirFile, O_READ)) {
            file.getName(first, MAXFILENAME_LENGTH);
            tft.println(first);
            num = false;
        } else if (file.openNext(&dirFile, O_READ)) {
            file.getName(filename, MAXFILENAME_LENGTH);
            tft.println(filename);
        } else {
            break;
        }
    }
    
    file.close();

    dirFile.close();  
}


void menu_init() {

    show_files();

    boolean romSelected = false;

    while (!romSelected) {

        readInputs();

        if (gb.direct.joypad_bits.start == 0) {
            romSelected = true;
        }

        if (gb.direct.joypad_bits.select == 0) {
            fileIndex = 0;
            show_files();
        }

        if (gb.direct.joypad_bits.up == 0) {
            fileIndex--;
            show_files();
        }

        if (gb.direct.joypad_bits.down == 0) {
            fileIndex++;
            show_files();
        }

        if (gb.direct.joypad_bits.right == 0) {
            fileIndex = fileIndex + MAXFILES;
            show_files();
        }

        if (gb.direct.joypad_bits.left == 0) {
            fileIndex = fileIndex - MAXFILES;
            show_files();
        }
    }
}

void sd_init() {
    if (!SD.begin(SD_CONFIG)) {
        printf("SD init failed!\n");
    }

    sprintf(PATH, (char * )"/");
}

void setup()
{
    video_init();

    sd_init();

    menu_init();

    strcpy(PATH, "/");
    strcat(PATH, first);
    strcat(PATH, "/");

    fileIndex = 0;

    menu_init();

    strcpy(ROM_PATH, PATH);
    strcat(ROM_PATH, first);

    flash_rom();

    enum gb_init_error_e gb_ret;
    printf("app_main running on core %d\n",xPortGetCoreID());
    printf("INIT: \n");

    // Initialise GameBoy emulator
    gb_ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, NULL);

    printf("GB: \n");
    switch(gb_ret)
    {
        case GB_INIT_NO_ERROR:
            break;
        case GB_INIT_CARTRIDGE_UNSUPPORTED:
            printf("gb_init returned an error: Unsupported cartridge.\n");
            break;
        case GB_INIT_INVALID_CHECKSUM:
            printf("gb_init returned an error: Checksum failure.\n");
            break;
        default:
             printf("gb_init returned an unknown error: %d\n", gb_ret);
    }

    gb_init_lcd(&gb, &lcd_draw_line);

    gb.direct.interlace = (char) tolower(0);
    gb.direct.frame_skip = (char) tolower(1);

    sound_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  gb_run_frame(&gb);
}