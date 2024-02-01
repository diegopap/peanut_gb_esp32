#ifndef _SOUND_H
#define _SOUND_H

typedef unsigned char u8;
typedef unsigned short int u16;
typedef signed short int s16;
typedef unsigned int u32;

//#define SQUARE_WAVE
#define TRIANGLE_WAVE

#ifdef __cplusplus
extern "C" {
#endif

// Memory IO

void audio_write(u8 addr, u8 val);
u8   audio_read(u8 addr);

// APU callback

u8 audio_update();

#ifdef __cplusplus
};
#endif


// NR10 sweep register

#define MAX_FREQ            2048

#define SWEEP_TIME_BITS 	(0x70)
#define SWEEP_TIME_OFFS		(4)
#define SWEEP_DIR_BIT 		(0x08)
#define SWEEP_DIR_OFFS 		(3)
#define SWEEP_SHIFT_BITS	(0x07)
#define SWEEP_SHIFT_OFFS 	(0)

typedef struct {
    u8  enabled;
    u16 freq;
    u8  time;
    u8  dir;
    u8  shift;
    u16 timer;
} SWEEP;


// NR11, NR21, NR41 len register

#define DUTY_BITS			(0xC0)
#define DUTY_OFFS 			(6)
#define SOUND_LEN_BITS 		(0x3F)
#define SOUND_LEN_OFFS 		(0)

typedef struct {
    u8 duty;
    u8 len;
} DUTY_LEN;

// NR12, NR22, NR42 envelope register

#define INIT_VOLUME_BITS 	(0xF0)
#define INIT_VOLUME_OFFS 	(4)
#define ENV_DIR_BIT			(0x08)
#define ENV_DIR_OFFS 		(3)
#define ENV_SWEEP_BITS 		(0x07)
#define ENV_SWEEP_OFFS 		(0)

typedef struct {
    u8  disabled;
    u8  volume;
    u8  dir;
    u8  period;
    u16 timer;
} ENVELOPE;

// NR13 & 14, NR23 & 24, NR33 & 34, NR44: init, counter, freq

#define FREQ_LO_MASK 		(0x00FF)
#define FREQ_HI_MASK 		(0xFF00)
#define INIT_BIT 			(0x80)
#define COUNTER_BIT 		(0x40)
#define COUNTER_OFFS 		(6)
#define FREQ_HI_BITS 		(0x07)

typedef struct {
    u8  enable;
    u8  initset;
    u8  counterset;
    u16 freq;
    u16 timer;
} CHANNEL;

// NR30, NR31

#define NR30_SOUND_ON_BIT	(0x80)
#define NR30_SOUND_ON_OFFS 	(7)
#define NR31_SOUND_LEN_BITS	(0xFF)
#define NR32_OUT_LEVEL_BITS	(0x60)
#define NR32_OUT_LEVEL_OFFS	(5)

// NR43

#define NR43_SHIFT_CLOCK_BITS	(0xF0)
#define NR43_SHIFT_CLOCK_OFFS 	(4)
#define NR43_COUNTER_STEP_BITS 	(0x08)
#define NR43_COUNTER_STEP_OFFS 	(3)
#define NR43_DIV_RATIO_BITS 	(0x03)
#define NR43_DIV_RATIO_OFFS 	(0)

// NR50 L/R output volume

#define NR50_L_ENABLE_BIT   (0x80)
#define NR50_L_ENABLE_OFFS  (7)
#define NR50_L_VOL_BITS     (0x70)
#define NR50_L_VOL_OFFS     (4)
#define NR50_R_ENABLE_BIT   (0x08)
#define NR50_R_ENABLE_OFFS  (3)
#define NR50_R_VOL_BIT      (0x07)
#define NR50_R_VOL_OFFS     (0)

// NR51 L/R channel mask

#define NR51_L_MASK_BITS    (0xF0)
#define NR51_L_MASK_OFFS    (4)
#define NR51_R_MASK_BITS    (0x0F)
#define NR51_R_MASK_OFFS    (0)

// NR52 audio power & status

#define NR52_ADDR           (0x26)
#define NR52_POWER_BIT      (0x80)
#define NR52_POWER_OFFS     (7)
#define NR52_STATUS_BITS    (0x0F)
#define NR51_STATUS_OFFS    (0)

// Audio RAM
#define AUDIO_RAM_SIZE          16

// Audio Channel 1 - Square Wave 1

typedef struct {
    SWEEP       sweep;      /* NR10 */
    DUTY_LEN    duty_len;   /* NR11 */
    ENVELOPE    envelope;    /* NR12 */
    CHANNEL     channel;    /* NR13, NR14 */
} CH1_t;

// Audio Channel 2 - Square Wave 2

typedef struct {
    DUTY_LEN    duty_len;   /* NR21 */
    ENVELOPE    envelope;    /* NR22 */
    CHANNEL     channel;    /* NR23, NR24 */
} CH2_t;

// Audio Channel 3 - DAC

typedef struct {
    u8          enable;     /* NR30 */
    u8          sound_len;  /* NR31 */
    u8          out_level;  /* NR32 */
    CHANNEL     channel;    /* NR33, NR34 */
    u8          pos_counter;
} CH3_t;

// Audio Channel 4 - Noise

typedef struct {
    DUTY_LEN    len;        /* NR41 */ // no duty
    ENVELOPE    envelope;   /* NR42 */
    u8          NR43;       /* NR43 */
    CHANNEL     channel;    /* NR44 */ // no freq
    u16         LFSR;
} CH4_t;

// Sound system settings

typedef struct {
    u8 power;
    u8 l_enable;    u8 l_vol;
    u8 r_enable;    u8 r_vol;
    u8 l_mask;      u8 r_mask;
} SO_t;

#endif // _SOUND_H
