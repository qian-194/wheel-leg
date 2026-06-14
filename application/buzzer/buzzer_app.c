#include "buzzer/buzzer_app.h"
#include "alarm/buzzer.h"
#include "string.h"

#define BUZZER_APP_TASK_PERIOD_MS 10U
#define BUZZER_APP_NOTE_OFF 0xFFU

typedef struct
{
    uint8_t octave;
    uint16_t duration_ms;
    float loudness;
} BuzzerAppNote_s;

static BuzzzerInstance *buzzer_instances[BUZZER_DEVICE_CNT];

static const BuzzerAppNote_s startup_sound[] = {
    {OCTAVE_5, 120, 0.30f},
    {BUZZER_APP_NOTE_OFF, 80, 0.0f},
    {OCTAVE_6, 160, 0.30f},
};

static const BuzzerAppNote_s remote_lost_sound[] = {
    {OCTAVE_3, 120, 0.35f},
    {BUZZER_APP_NOTE_OFF, 80, 0.0f},
    {OCTAVE_3, 120, 0.35f},
    {BUZZER_APP_NOTE_OFF, 80, 0.0f},
    {OCTAVE_3, 220, 0.35f},
};

static const BuzzerAppNote_s remote_recover_sound[] = {
    {OCTAVE_5, 100, 0.25f},
    {BUZZER_APP_NOTE_OFF, 60, 0.0f},
    {OCTAVE_7, 180, 0.25f},
};

static const BuzzerAppNote_s error_sound[] = {
    {OCTAVE_1, 500, 0.45f},
};

static const BuzzerAppNote_s *active_sound;
static uint8_t active_sound_len;
static uint8_t active_note_idx;
static uint16_t active_note_elapsed_ms;

static void BuzzerAppStopSound(void)
{
    if (buzzer_instances[ALARM_LEVEL_HIGH] != NULL)
    {
        AlarmSetStatus(buzzer_instances[ALARM_LEVEL_HIGH], ALARM_OFF);
    }
}

static void BuzzerAppApplyNote(const BuzzerAppNote_s *note)
{
    BuzzzerInstance *sound_buzzer = buzzer_instances[ALARM_LEVEL_HIGH];

    if ((sound_buzzer == NULL) || (note == NULL) || (note->octave == BUZZER_APP_NOTE_OFF))
    {
        BuzzerAppStopSound();
        return;
    }

    sound_buzzer->octave = (octave_e)note->octave;
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
