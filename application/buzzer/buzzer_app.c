#include "buzzer/buzzer_app.h"
#include "alarm/buzzer.h"
#include "string.h"

#define BUZZER_APP_TASK_PERIOD_MS 10U
#define BUZZER_APP_FREQ_FROM_OCTAVE 0U
#define BZR_REST 0U
#define NOTE_C4 262U
#define NOTE_D4 294U
#define NOTE_E4 330U
#define NOTE_G4 392U

typedef struct
{
    uint8_t octave;
    uint16_t frequency_hz;
    uint16_t duration_ms;
    float loudness;
} BuzzerAppNote_s;

static BuzzzerInstance *buzzer_instances[BUZZER_DEVICE_CNT];

static const BuzzerAppNote_s startup_sound[] = {
    {OCTAVE_5, BUZZER_APP_FREQ_FROM_OCTAVE, 120, 0.30f},
    {OCTAVE_1, BZR_REST, 80, 0.0f},
    {OCTAVE_6, BUZZER_APP_FREQ_FROM_OCTAVE, 160, 0.30f},
};

static const BuzzerAppNote_s remote_lost_sound[] = {
    {OCTAVE_3, BUZZER_APP_FREQ_FROM_OCTAVE, 120, 0.35f},
    {OCTAVE_1, BZR_REST, 80, 0.0f},
    {OCTAVE_3, BUZZER_APP_FREQ_FROM_OCTAVE, 120, 0.35f},
    {OCTAVE_1, BZR_REST, 80, 0.0f},
    {OCTAVE_3, BUZZER_APP_FREQ_FROM_OCTAVE, 220, 0.35f},
};

static const BuzzerAppNote_s remote_recover_sound[] = {
    {OCTAVE_5, BUZZER_APP_FREQ_FROM_OCTAVE, 100, 0.25f},
    {OCTAVE_1, BZR_REST, 60, 0.0f},
    {OCTAVE_7, BUZZER_APP_FREQ_FROM_OCTAVE, 180, 0.25f},
};

static const BuzzerAppNote_s error_sound[] = {
    {OCTAVE_1, BUZZER_APP_FREQ_FROM_OCTAVE, 500, 0.45f},
};

static const BuzzerAppNote_s nmpc_timeout_sound[] = {
    {OCTAVE_4, BUZZER_APP_FREQ_FROM_OCTAVE, 70, 0.35f},
    {OCTAVE_1, BZR_REST, 60, 0.0f},
    {OCTAVE_4, BUZZER_APP_FREQ_FROM_OCTAVE, 70, 0.35f},
    {OCTAVE_1, BZR_REST, 60, 0.0f},
    {OCTAVE_4, BUZZER_APP_FREQ_FROM_OCTAVE, 70, 0.35f},
};

static const BuzzerAppNote_s see_you_again_sound[] = {
    // When I see you again: 1, 2, 3, 5, 3, 2
    {OCTAVE_1, NOTE_C4, 375, 0.30f},
    {OCTAVE_1, NOTE_D4, 375, 0.30f},
    {OCTAVE_1, NOTE_E4, 1875, 0.30f},
    {OCTAVE_1, NOTE_G4, 375, 0.30f},
    {OCTAVE_1, NOTE_E4, 750, 0.30f},
    {OCTAVE_1, NOTE_D4, 1500, 0.30f},
};

static const BuzzerAppNote_s *active_sound;
static uint8_t active_sound_len;
static uint8_t active_note_idx;
static uint16_t active_note_elapsed_ms;

static void BuzzerAppStopSound(void)
{
    if (buzzer_instances[ALARM_LEVEL_HIGH] != NULL)
    {
        buzzer_instances[ALARM_LEVEL_HIGH]->frequency_hz = BUZZER_APP_FREQ_FROM_OCTAVE;
        AlarmSetStatus(buzzer_instances[ALARM_LEVEL_HIGH], ALARM_OFF);
    }
}

static void BuzzerAppApplyNote(const BuzzerAppNote_s *note)
{
    BuzzzerInstance *sound_buzzer = buzzer_instances[ALARM_LEVEL_HIGH];

    if ((sound_buzzer == NULL) || (note == NULL) || (note->frequency_hz == BZR_REST))
    {
        BuzzerAppStopSound();
        return;
    }

    sound_buzzer->octave = (octave_e)note->octave;
    sound_buzzer->frequency_hz = note->frequency_hz;
    sound_buzzer->loudness = note->loudness;
    AlarmSetStatus(sound_buzzer, ALARM_ON);
}

void BuzzerAppInit(void)
{
    for (uint8_t i = 0; i < BUZZER_DEVICE_CNT; i++)
    {
        Buzzer_config_s config = {
            .alarm_level = (AlarmLevel_e)i,
            .octave = OCTAVE_5,
            .frequency_hz = BUZZER_APP_FREQ_FROM_OCTAVE,
            .loudness = 0.0f,
        };

        buzzer_instances[i] = BuzzerRegister(&config);
        AlarmSetStatus(buzzer_instances[i], ALARM_OFF);
    }

    active_sound = NULL;
    active_sound_len = 0;
    active_note_idx = 0;
    active_note_elapsed_ms = 0;
}

void BuzzerAppPlay(BuzzerAppSound_e sound)
{
    switch (sound)
    {
    case BUZZER_APP_SOUND_STARTUP:
        active_sound = startup_sound;
        active_sound_len = sizeof(startup_sound) / sizeof(startup_sound[0]);
        break;
    case BUZZER_APP_SOUND_SEE_YOU_AGAIN:
        active_sound = see_you_again_sound;
        active_sound_len = sizeof(see_you_again_sound) / sizeof(see_you_again_sound[0]);
        break;
    case BUZZER_APP_SOUND_REMOTE_LOST:
        active_sound = remote_lost_sound;
        active_sound_len = sizeof(remote_lost_sound) / sizeof(remote_lost_sound[0]);
        break;
    case BUZZER_APP_SOUND_REMOTE_RECOVER:
        active_sound = remote_recover_sound;
        active_sound_len = sizeof(remote_recover_sound) / sizeof(remote_recover_sound[0]);
        break;
    case BUZZER_APP_SOUND_ERROR:
        active_sound = error_sound;
        active_sound_len = sizeof(error_sound) / sizeof(error_sound[0]);
        break;
    case BUZZER_APP_SOUND_NMPC_TIMEOUT:
        active_sound = nmpc_timeout_sound;
        active_sound_len = sizeof(nmpc_timeout_sound) / sizeof(nmpc_timeout_sound[0]);
        break;
    default:
        active_sound = NULL;
        active_sound_len = 0;
        BuzzerAppStopSound();
        return;
    }

    active_note_idx = 0;
    active_note_elapsed_ms = 0;
    BuzzerAppApplyNote(&active_sound[active_note_idx]);
}

void BuzzerAppTask(void)
{
    if (active_sound == NULL)
    {
        return;
    }

    active_note_elapsed_ms += BUZZER_APP_TASK_PERIOD_MS;

    if (active_note_elapsed_ms < active_sound[active_note_idx].duration_ms)
    {
        return;
    }

    active_note_elapsed_ms = 0;
    active_note_idx++;

    if (active_note_idx >= active_sound_len)
    {
        active_sound = NULL;
        active_sound_len = 0;
        active_note_idx = 0;
        BuzzerAppStopSound();
        return;
    }

    BuzzerAppApplyNote(&active_sound[active_note_idx]);
}
