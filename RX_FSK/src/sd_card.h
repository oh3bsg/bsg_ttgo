// sd_card module header
#ifndef _SD_CARD
#define _SD_CARD

void sd_card_init(void);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void createDir(fs::FS &fs, const char * path);
void sd_card_log(SondeInfo *s);

#endif // _SD_CARD