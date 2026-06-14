#ifndef APPLICATION_BUZZER_APP_H
#define APPLICATION_BUZZER_APP_H

typedef enum
{
    BUZZER_APP_SOUND_STARTUP = 0,
    BUZZER_APP_SOUND_REMOTE_LOST,
    BUZZER_APP_SOUND_REMOTE_RECOVER,
    BUZZER_APP_SOUND_ERROR,
} BuzzerAppSound_e;

void BuzzerAppInit(void);
void BuzzerAppTask(void);
void BuzzerAppPlay(BuzzerAppSound_e sound);

#endif
