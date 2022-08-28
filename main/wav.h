#ifndef __WAV_H__
#define __WAV_H__

void wav_start(int out);
void wav_stop (void);
int  wav_read (void *pcm, int size);
int  wav_write(void *pcm, int size);

#endif

