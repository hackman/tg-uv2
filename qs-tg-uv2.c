
/*

   Quansheng TG-UV2 Utility by Mike Nix <mnix@wanm.com.au>

   Currently only reads the device memory, and displays the contents nicely formatted.
   
   
   Supports:
      All channel config, except CTSS/DCS settings (we know where they are, just haven't
      bothered to interpret them yet)

      All the settings configurable via Options in the original windows utility.

      Both of the known keypad dances for enabling/disabling bands are detected/displayed.
      and yes, we can do the same thing by updating the config.

      Saves the config read from the device to a file (tguv2.dat)
        **** THIS IS NOT THE SAME FORMAT USED BY THE WINDOWS UTILITY ****
      The windows utility saves in some proprietry format... we just dump
      a copy of the radio memory to disk :)

      Reads a file saved above, and uploads it to the radio.      
      
   TODO:
      There are at least 5 memory locations that do stuff I can't figure out (config->unk*)
      
      I never found a way to enable transmit on the 470-520 band
      (Australian UHF CB is at 476-477 Mhz).  Probably need a firmware hack for this :(
      
      Command-line options for settings so we can change the config easily.

      Find a copy of the firmware, and the update utility so we can really open it up.


   There is a command line option to manually set any byte in the config area for
   testing purposes, but this is not the way to configure your radio normally :)

   There is another command line option for sending arbitrary data to the radio in 
   programming mode...
   

   Other Info:
   -----------
   There are positive responses (ACK) to the following commands if followed
   by any other byte:
      G, O, P, Q  but have no idea what they do....
   
   The known commands for programming and getting the model number are:
   \002PnOGdAM	Enter Programming Mode
   M\002	Get Model Number? (Returns 'P5555' followed by F4 00 00)
   R		Read config memory
   W		Write config memory
   
   Any 1-byte after entering program mode followed by 0x02 will return the model number
   
   Other commands will not work until after you get the model number.

   R command is followed by addr_hi, addr_lo, length
   W command is followed by addr_hi, addr_lo, length then data bytes

   A null byte is sent by the radio every time the display changes... no idea what use that is.
   
   Further testing reveals that any 8 bytes written will do to start programming mode.
   ** maybe there is a special sequence for firmware mode?
   Actually, any sequence of more than 1 byte containing "M" seems to hit program mode.
     
*/


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

// Quansheng TG-UV2 uses 9600,N,8,2
#define RADIO_BAUD B9600

#define ACK 0x06

#define TGUV2_MEMSIZE 0x2000
#define TGUV2_CHANNELS 200
#define TGUV2_BANDS 8

// shorthand forms of integer types for convenience
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#pragma pack(1)
struct tguv2_channel {
   u32	freq; // frequency in 10s of Hz, stored in BCD
   u32	txofs;// offset to tx frequency, 10s of Hz, stored in BCD
   u8   rc_lo;
   u8	tc_lo;
   u8	code_type;  // low nibble is rc_hi, high nibble is tc_hi
                    // or could be type number (0-3)
   u8	flags1;
   u8	flags2;
   u8	flags3;    // always 0xFF ??
   u8	bandwidth; // channel step / bandwidth - 6=25, 9=100
   u8	tx_power;
};

struct tguv2_channel2 { // extra channel data
   u8   name[6];
   u8	data[10];
};

#define TXPOWER_LOW 2
#define TXPOWER_MID 1
#define TXPOWER_HI  0

#define CF1_SHIFT_POS	0x01
#define CF1_SHIFT_NEG	0x02
#define CF2_REVERSE_ON	0x01
#define CF2_SCRAMBLE_ON	0x02
#define CF2_WideFM	0x10

struct vfo {
   u8 current; // memory slot for this vfo: 0-199 = channel mode, 200-204=vfo band
   u8 chan;    // last used channel no
   u8 memno;   // last used band (as mem slot number 200-204)
};

struct tguv2_config {
   struct tguv2_channel	channels[TGUV2_CHANNELS];	// the channel memories
   struct tguv2_channel bands[TGUV2_BANDS];		// current / most recent settings for each band
   u8 cflags[TGUV2_CHANNELS];				// scan flags and band numbers
   u8 bandid[TGUV2_BANDS];				// band ids (F0 - F4)
   u8 reserved1[0x30];					// unused?, all 0xFF

   u8 unk1; // 00
   u8 squelch;						// 0 - 9
   u8 time_out_timer;					// 01 (minutes)
   u8 priority_channel;
   
   u8 keylock;						// 01=off, 00=on
   u8 busy_lockout;					// 01=off, 00=on
   u8 vox;						// 00=off, 1-9
   u8 unk2; // FF
   
   u8 keybeep;						// 00=on, 01=off
   u8 display;						// 00=Freq, 01=Channel, 02=Name
   u8 step; 						// FF=5, ??
   u8 unk3; // FF
   
   u8 unk4; // 00
   u8 mode;						// 00=DualWatch, 01=cross-band, other=off
   u8 end_tone;						// 0=on, 1=off
   u8 vfo_model;					// 0=VFO Disabled, other=VFO Enabled
   
   struct vfo vfo[2];
   
   u8 unk5; // 00
   u8 reserved2[9];					// Unused?, ALL 0xFF
   
   // band_restrict can be toggled by holding PTT+MON at Power on, until you hear a second beep
   u8 band_restrict;					// !=01 = all bands enabled
   
   // this can be toggled by holding BAND during power on until the display is ******
   // then enter 350390 on the keypad.
   u8 txen350390;					// FF=unset, 00=DIS, 01=EN (!=01 == DIS) 
   
   u8 reserved3[0xDE];					// Unused?, ALL 0xFF

   struct tguv2_channel2 channels2[TGUV2_CHANNELS];	// channel names etc
};

#define CFLAG_SCAN	0x80
#define CFLAG_BANDMASK	0x07

#pragma pack()

#define PMODE_NONE	-1
#define PMODE_PROGRAM	2
#define PMODE_FIRMWARE	1

double bandwidthmap[]={ 5.0, 6.25, 10.0, 12.5, 15, 20, 25, 30, 50, 100, 101, 102, 103, 104, 105, 5.0 };

// current radio state
int radio_mode=PMODE_NONE;

// Serial port options.
int debug=3;
int TX_enabled=0;

int TX_auto_on=1;
int RX_auto_on=1;


void hexdump(unsigned char *data, int len)
{
  int i;
  for (i=0; i<len; i++)
      printf(" %02x", data[i]);
  printf("  ");
  for (i=0; i<len; i++)
      printf("%c",  isprint(data[i]) ? data[i] : '.');
}

void TX_on(int fd)
{
  // raise RTS
  if (TX_auto_on) {
     // hardware works this out automatically so don't do anything
     }
  else {
     int status;
     if (debug>2) printf("RTS ON\r\n");
     ioctl(fd, TIOCMGET, &status);
     status |= TIOCM_RTS;
     if (ioctl(fd, TIOCMSET, &status)) {
        perror("ioctl");
        }
     }
  TX_enabled=1;
}

void TX_off(int fd)
{
  // lower RTS
  if (TX_auto_on) {
     // hardware does this automatically so don't do anything
     }
  else {
     int status;
     if (debug>2) printf("RTS OFF\r\n");
     tcdrain(fd);
     ioctl(fd, TIOCMGET, &status);
     status &= ~TIOCM_RTS;
     ioctl(fd, TIOCMSET, &status);
     }
  TX_enabled=0;
}

void RX_on(int fd)
{
  // raise DTR
  if (RX_auto_on) {
     }
  else {
     int status;
     if (debug>2) printf("DTR ON\r\n");
     ioctl(fd, TIOCMGET, &status);
     status |= TIOCM_DTR;
     if (ioctl(fd, TIOCMSET, &status)) {
        perror("ioctl");
        }
     }
}

void RX_off(int fd)
{
  // lower DTR
  if (RX_auto_on) {
     }
  else {
     int status;
     if (debug>2) printf("DTR OFF\r\n");
     ioctl(fd, TIOCMGET, &status);
     status &= ~TIOCM_DTR;
     ioctl(fd, TIOCMSET, &status);
     }
}


void TX(int fd, unsigned char *data, int len)
{
  if (!TX_enabled) TX_on(fd);
  
  if (debug>3) {
     printf("TX:");
     hexdump(data,len);
     printf("\r\n");
     }
  write(fd, data, len);
}

int RX(int fd, unsigned char *data, int len)
{
  int rc = read(fd, data, len);
   
  if (debug>3 && rc>0) {
     printf("RX:");
     hexdump(data,rc);
     printf("\r\n");
     }
  return rc;
}


#define TEST1

int start_pgm_mode(int fd, int pmode)
{
  u8 buf[8];

  if (pmode==PMODE_FIRMWARE) {
     buf[0]=0x02;
     buf[1]='F';
     buf[2]='i';
     buf[3]='r';
     buf[4]='m';
     buf[5]='w';
     buf[6]='a';
     buf[7]='r';
     }
  else {
#ifndef TEST1
     buf[0]=0x02;
     buf[1]=0x50;   // "PnOGdAM"
     buf[2]=0x6E;
     buf[3]=0x4F;
     buf[4]=0x47;
     buf[5]=0x64;
     buf[6]=0x41;
     buf[7]=0x4D;
#else
     buf[0]=0x02;
     buf[1]='P';   // "PnOGdAM"
     buf[2]='n';
     buf[3]='O';
     buf[4]='G';
     buf[5]='d';
     buf[6]='A';
     buf[7]='M';
#endif
     }
     
  TX(fd, buf, 8);
  
  if (RX(fd, buf, 1)==1) {
     if (buf[0]==ACK) {
        printf("Radio detected\n");
        radio_mode=pmode;
        return 0;
        }
     printf("Bad response from radio: 0x%02x\n", buf[0]);
     return -ENODEV;
     }
  printf("No response from radio\n");
  return -ENODEV;
}

int read_model(int fd, u8 *model) {
  u8 buf[8];
  int rc;
#ifndef TEST1
  buf[0]=0x4D; // 'M'
  buf[1]=0x02;
#else
  buf[0]='M'; // 'M'
  buf[1]=0x02;
#endif
  TX(fd, buf, 2);
  rc=RX(fd, model, 8);
  if (rc!=8) {
     printf("Error reading model data, got %d bytes:", rc);
     if (rc>0) hexdump(model,rc);
     printf("\n");
     return -1;
     }
  buf[0]=ACK;
  TX(fd, buf, 1);
  if (RX(fd, buf, 1)!=1) {
     printf("Didn't get expected ACK response\n");
     return -1;
     }
  if (buf[0]!=ACK) {
     printf("Expected ACK, got 0x%02x\n", buf[0]);
     }
  printf("Model: ");
  hexdump(model, 8);
  printf("\r\n");
  return 0;
}

int read_block(int fd, void *dest, u8 *buf, u16 addr, u8 len) {
  
  printf("Reading memory 0x%04x\r", addr);
  memset(buf, 0, 12);
  buf[0]='R';
  buf[1]=(addr >> 8) & 0xFF;
  buf[2]=addr & 0xFF;
  buf[3]=len;
  TX(fd, buf, 4);
  if (RX(fd, buf, len+4)==len+4) {
     if (buf[0]==0x57 && buf[3]==len &&
         (buf[1]<<8 | buf[2]) == addr
        ) {
        memcpy(dest, buf+4, len);
        return 1;
        }
     else {
        printf("\nunexpected data for address 0x%04x\n", addr);
        }
     } 
  else {
     printf("\nFailed read for address 0x%04x\n", addr);
     }
  return 0;
}

int read_block2(int fd, void *dest, u8 *buf, u32 addr, u8 len) {
  
  printf("Reading memory 0x%04x\r", addr);
  memset(buf, 0, 12);
  buf[0]='R';
  buf[1]=(addr >> 24) & 0xFF;
  buf[2]=(addr >> 16) & 0xFF;
  buf[3]=(addr >> 8) & 0xFF;
  buf[4]=addr & 0xFF;
  buf[5]=len;
  TX(fd, buf, 6);
  if (RX(fd, buf, len+4)==len+4) {
     if (buf[0]==0x57 && buf[5]==len &&
         (buf[3]<<8 | buf[4]) == addr
        ) {
        memcpy(dest, buf+4, len);
        return 1;
        }
     else {
        printf("\nunexpected data for address 0x%04x\n", addr);
        }
     } 
  else {
     printf("\nFailed read for address 0x%04x\n", addr);
     }
  return 0;
}

int write_block(int fd, u8 *src, u16 addr, u8 len) {

  u8 buf[4];
  
  printf("Writing memory 0x%04x\r", addr);
  buf[0]='W';
  buf[1]=(addr >> 8) & 0xFF;
  buf[2]=addr & 0xFF;
  buf[3]=len;
  TX(fd, buf, 4);
  TX(fd, src, len);
  if (RX(fd, buf, 1)==1) {
     if (buf[0]!=ACK)
        printf("Expected ACK, got 0x%02x for address 0x%04d\n", buf[0], addr);
     }
  else {
     printf("\nDid not receive ACK for address 0x%04x\n", addr);
     }
}


int test_cmd(int fd, unsigned char *cmd, u8 sendlen, u8 expectlen) {

  printf("Radio Mode: ");
  switch (radio_mode) {
     case PMODE_NONE:		printf("NONE");     break;
     case PMODE_PROGRAM:	printf("PROGRAM");  break;
     case PMODE_FIRMWARE:	printf("FIRMWARE"); break;
     default : printf("Unknown: %d", radio_mode);   break;
     }
  printf("\n");

  printf("Test Command: (%d bytes) ", sendlen);
  hexdump(cmd, sendlen);
  printf("\n");
  TX(fd, cmd, sendlen);

  int dbg=debug;
  debug=4;
  int rc=RX(fd, cmd, expectlen);
  debug=dbg;
  printf("Test Command: RX returned %d (expected %d bytes)\n", rc, expectlen);
  return 0;
}


int read_memory(int fd, struct tguv2_config *config, int setuponly) {
  int addr, end;
  u8 buf[12];
  u8 *data=(u8*)config;
  
  addr=0;
  end=TGUV2_MEMSIZE;
  if (setuponly) {
     addr=0x0C80;
     end =0x0F00;
     }
     
  for (; addr < end; addr += 8) {
      read_block(fd, data+addr, buf, addr, 8);
      }
  return TGUV2_MEMSIZE;
}


int write_memory(int fd, struct tguv2_config *config, int setuponly) {
  int addr, end;
  u8 *data=(u8*)config;
  
  addr=0;
  end=TGUV2_MEMSIZE;
  if (setuponly) {
     addr=0x0C80;
     end =0x0F00;
     }

  for (; addr < end; addr += 8) {
      write_block(fd, data+addr, addr, 8);
      }
  return TGUV2_MEMSIZE;
}

int read_channel(int fd, struct tguv2_config *config, u8 channel) {\
  int addr, end;
  u8 buf[12];
  u8 *data;

  if (radio_mode != PMODE_PROGRAM) {
     printf("read_channel called with radio not in PROGRAM mode\n");
     return;
     }
  
  // read the main info
  data=(u8*)(&config->channels[channel]);
  addr=data - (u8*)config;
  read_block(fd, data, buf, addr, 8);
  read_block(fd, data+8, buf, addr+8, 8);

  // read the extra info (name)
  data=(u8*)(&config->channels2[channel]);
  addr=data - (u8*)config;
  read_block(fd, data, buf, addr, 8);
  read_block(fd, data+8, buf, addr+8, 8);
  
}

int write_channel(int fd, struct tguv2_config *config, u8 channel) {\
  int addr, end;
  u8 *data;

  if (radio_mode != PMODE_PROGRAM) {
     printf("write_channel called with radio not in PROGRAM mode\n");
     return;
     }
  
  // read the main info
  data=(u8*)(&config->channels[channel]);
  addr=data - (u8*)config;
  write_block(fd, data, addr, 8);
  write_block(fd, data+8, addr+8, 8);

  // read the extra info (name)
  data=(u8*)(&config->channels2[channel]);
  addr=data - (u8*)config;
  write_block(fd, data, addr, 8);
  write_block(fd, data+8, addr+8, 8);
  
}

void save_memory(char *fname, struct tguv2_config *data)
{
  int fd=open(fname, O_CREAT | O_WRONLY, 0660);
  write(fd, data, TGUV2_MEMSIZE);
  close(fd);
}

void load_memory(char *fname, struct tguv2_config *data)
{
  int fd=open(fname, O_RDONLY, 0660);
  read(fd, data, TGUV2_MEMSIZE);
  close(fd);
}

u32 bcdtoint(u32 bcd)
{
  u32 i=0;
  int j;
  bcd = be32toh(bcd);
  for (j=7; j>=0; j--) {
      i+=(bcd >> (4*j)) & 0x0F;
      i*=10;
      }
  return i;
}

const char charmap[]="0123456789ABCDEFGHIJKLMNOPWRSTUVWXYZ |* +-\0";

char *channel_name(u8 *enc, u8 len)
{
  u8 i;
  static char buf[40];
  for (i=0; i<len; i++) {
      if (enc[i] <= 43) 
              buf[i]=charmap[enc[i]];
         else buf[i]='.';
      }
  buf[i++]=0;
  return buf;
}

void name_channel(u8 *enc, char *name, u8 len)
{
  int i,j;
  for (i=0; i<len && *name; i++, name++) {
      for (j=0; charmap[j] && charmap[j]!=*name; j++);
      if (charmap[j]) *enc++=j;
      }
}

void print_channel(struct tguv2_channel *chan)
{
  u32 rxf, txf;

  rxf = bcdtoint(chan->freq);
  txf = rxf;
  if (chan->flags1 & CF1_SHIFT_POS) 
     txf += bcdtoint(chan->txofs);
  if (chan->flags1 & CF1_SHIFT_NEG)
     txf -= bcdtoint(chan->txofs);

  printf("%10.6f  %10.6f  %6.2f ",
         rxf / 1000000.0,
         txf / 1000000.0,
         bandwidthmap[chan->bandwidth & 0x0F]);
     
  printf("%-3s %-3s %-4s  ",
         (chan->flags2 & CF2_SCRAMBLE_ON ? "OFF" : "ON"),
         (chan->flags2 & CF2_REVERSE_ON ? "OFF" : "ON"),
         (chan->flags2 & CF2_WideFM ? "Wide" : "Narr"));
         
  switch (chan->tx_power) {
         case TXPOWER_LOW: printf("LOW"); break;
         case TXPOWER_MID: printf("MID"); break;
         case TXPOWER_HI : printf("HI "); break;
         default: printf("???");
         }

}
    
void list_channels(struct tguv2_config *conf)
{
  int c;
  
  printf(" CH   RX          TX          Step  Scr REV Wide  Pow  SCN  BAND  CH_Name\n");
  for (c=0; c<TGUV2_CHANNELS; c++) {
      if (conf->channels[c].freq != 0xFFFFFFFF) {
         printf("%3d: ", c);
         print_channel(&(conf->channels[c]));

         printf("  %-3s   %02X",
            conf->cflags[c] & CFLAG_SCAN ? "Yes" : "No",
            conf->bandid[conf->cflags[c] & CFLAG_BANDMASK]);

         printf("  %s\n", channel_name(conf->channels2[c].name, 6));
 
         }
      }

}


void list_bands(struct tguv2_config *conf)
{
  int b;
  printf("          RX          TX          Step  Scr REV Wide  Pow\n");
  for (b=0; b<TGUV2_BANDS; b++) {
      if (conf->bands[b].freq != 0xFFFFFFFF) {
         printf("Band %02X: ", conf->bandid[b]);
         print_channel(&(conf->bands[b]));
         printf("\n");
         }
      }
}


void list_config(struct tguv2_config *conf)
{
  printf("Squelch         : %d\n", conf->squelch);
  printf("Time-Out-Timer  : %d seconds\n", conf->time_out_timer*60);
  printf("Priority Channel: ");
  if (conf->priority_channel < 200) printf("%d\n", conf->priority_channel);
                               else printf("OFF\n");
  printf("Busy Lockout    : %s  (%02x)\n", conf->busy_lockout==0x01 ? "OFF" : "ON", conf->busy_lockout);
  printf("VOX             : %d\n", conf->vox);
  printf("Key Lock        : %s\n", conf->keylock==0x01 ? "OFF" : "ON");
  printf("Key Beep        : %s\n", conf->keybeep==0x01 ? "OFF" : "ON");
  printf("Freq Step       : %1.2f  (%02x)\n", bandwidthmap[conf->step & 0x0F], conf->step & 0x0F, conf->step);
  printf("End Tone        : %s\n", conf->end_tone ? "OFF" : "ON");
  printf("VFO Model       : %s  (%02x)\n", conf->vfo_model==0x00 ? "No" : "Yes", conf->vfo_model);

  printf("Mode            : ");
  switch (conf->mode) {
     case 0x00: printf("Dual Watch"); break;
     case 0x01: printf("Cross-Band"); break;
     default:   printf("Normal (Channel/VFO)  (%02x)", conf->mode);
     }
  printf("\n");

  printf("Channel Display : ");
  switch (conf->display) {
     case 0x00: printf("Ch/Frequency"); break;
     case 0x01: printf("Channel");      break;
     case 0x02: printf("Ch/Name");      break;
     default:   printf("???  (%02x)", conf->display);
     }
  printf("\n");
  
  int v;
  for (v=0; v<2; v++) {
      if (conf->vfo[v].current < 200) {
         printf("VFO%d            : Channel %d\n", v, conf->vfo[v].chan);
         }
      else {
         printf("VFO%d            : Band %02X, Freq %10.6f\n",
                v,
                conf->bandid[conf->vfo[v].memno - 200],
                bcdtoint(conf->bands[conf->vfo[v].memno - 200].freq)/1000000.0);
         }
      }
      
  printf("Band Restrict   : %s\n", conf->band_restrict == 0x01 ? "Enabled" : "Disabled");
  printf("350-390 Mhz TX  : %s\n", conf->txen350390 == 0x01 ? "Enabled" : "Disabled");
  printf("test: %02x %02x\n", conf->band_restrict, conf->txen350390);
  
  printf("Unknown Bytes   : %02X %02X %02X %02X %02X\n",
        conf->unk1, conf->unk2, conf->unk3, conf->unk4, conf->unk5);
        
  printf("Reserved Bytes  :\n1: ");
  hexdump(conf->reserved1, sizeof(conf->reserved1));
  printf("\n2: ");
  hexdump(conf->reserved2, sizeof(conf->reserved2));
  printf("\n3: ");
  hexdump(conf->reserved3, sizeof(conf->reserved3));
  printf("\n");
}

int openserial(char *port)
{
  struct termios sp;
  if (debug>1) printf("Opening %s\r\n", port);
  
  // open non-blocking otherwise open can block until carrier appears
  int fd = open(port, O_RDWR | O_NONBLOCK);
  if (fd<0) return -ENODEV; 

  memset(&sp,0,sizeof(sp));
  sp.c_iflag=0;
  sp.c_oflag=0;
  sp.c_cflag=CS8|CREAD|CLOCAL|CSTOPB;
  sp.c_lflag=0;
  sp.c_cc[VMIN]=100;
  sp.c_cc[VTIME]=5;
  cfsetispeed(&sp,RADIO_BAUD);
  cfsetospeed(&sp,RADIO_BAUD);

  tcsetattr(fd, TCSANOW, &sp);
  if (debug>3) printf("tcflush()\r\n");

  // now switch to blocking IO because that's what we want
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
  
  tcflush(fd, TCIOFLUSH);
  return fd;
}


void help()
{
  printf("Quansheng TG-UV2 Radio control tool 0.1\n");
  printf("Usage: radio <options>\n");
  printf("	-r f	read config from file f\n");
  printf("	-w f	write config to file f\n");
  printf("	-g	get config from radio\n");
  printf("	-p	put config to radio\n");
  printf("	-l	list current config\n");	
  printf("	-c	list current channels\n");
  printf("\n");
  printf("To select a port other than /dev/ttyUSB0 specify the name as the first option\n");
  
  printf("\n\n");
  printf("Debugging Tools: (DO NOT USE)\n");
  printf("	-s addr value		- set config byte at addr to value\n");
  printf("	-f addr value len	- fill len bytes from addr with value\n");
  printf("	-e len			- expect len byte reply to test command\n");
  printf("	-T <data>		- Test command in program mode\n");
  printf("	-t <data>		- Test command.\n");
  printf("	-F <data>		- Test command in firmware mode ** DANGEROUS **\n");
  printf("		<data> consists of a length byte, followed by length characters\n");
  printf("			or numeric values (escape digits with \\)\n");
  printf("	-D addr len		- dump a chunk of memory\n");
}


int init_radio(int *serial, char *port, u8 *model, int pmode) {
  if (*serial<0) {
     *serial=openserial(port);
     if (*serial < 0) {
        perror("Error opening port");
        return 0;
        }
     RX_on(*serial);

     if (start_pgm_mode(*serial, pmode)==0 && read_model(*serial, model)==0) {
        return 1;
        }
     close(*serial);
     *serial=-1;
     return 0;
     }
  if (radio_mode != pmode) {
     printf("Can't change programming mode during a run.\n");
     printf("You can not mix and match firmware updates and programming updates\n");
     return 0;
     }
  return 1;
}

main (int argc, char *argv[])
{
  struct timeval tv;
  fd_set rfds, tfds;
  int retval;
  int quit=0;
  int i;
  int setaddr=-1;
  int setvalue=0x00;
  int setlen=0;
  
  unsigned char cmd_buf[20];
  int cmd_len=0;
  int cmd_expect=8;
  
  char *port="/dev/ttyUSB0";
  int serial = -1;

  u8 model[20];
  int config_valid=0, channels_valid=0;
  struct tguv2_config *config;
  
  if (sizeof(*config) > TGUV2_MEMSIZE) {
     printf("Program Error: tguv2_config structure is too big for device memory!\n");
     printf("   sizeof(config) = %d, TGUV2_MEMSIZE=%d\n", sizeof(*config), TGUV2_MEMSIZE);
     exit(0);
     }
  
  config = malloc(TGUV2_MEMSIZE);

  i=1;
  while (i<argc) {
     if (strncmp(argv[i],"/dev/",5)==0) {
        port=argv[i];
        }
     else if (strcmp(argv[i],"-h")==0) {
        help();
        quit=1;
        }
     else if (strcmp(argv[i],"-q")==0) {
        quit=1;
        }
     else if (strcmp(argv[i],"-r")==0) { // read config from file
        i++;
        load_memory(argv[i], config);
        config_valid=1;
        channels_valid=1;
        }
     else if (strcmp(argv[i],"-w")==0) { // write config to file
        i++;
        if (!config_valid && channels_valid) {
           printf("Write config to file: load a config with -r or -g first\n");
           }
        else {
           save_memory(argv[i], config);
           }
        }
     else if (strcmp(argv[i],"-g")==0) { // get config from radio
        if (init_radio(&serial, port, model, PMODE_PROGRAM)) {
           read_memory(serial, config, 0);
           config_valid=1;
           channels_valid=1;
           }
        }
     else if (strcmp(argv[i],"-p")==0) { // put config to radio
        if (!config_valid && channels_valid) {
           printf("Put Config: load a config with -r or -g first\n");
           }
        else if (init_radio(&serial, port, model, PMODE_PROGRAM)) {
           write_memory(serial, config, 0);
           }
        }
     else if (strcmp(argv[i], "-c")==0) { // list channels in loaded config
        if (config_valid) {
           list_channels(config);
           }
        else {
           printf("List Channels: load a config with -r or -g first\n");
           }
        }
     else if (strcmp(argv[i], "-l")==0) { // list loaded config
        if (config_valid) {
           list_bands(config);
           list_config(config);
           }
        else {
           printf("List Config: load a config with -r or -g first\n");
           }
        }
     else if (strcmp(argv[i],"-s")==0) { // set config bytes
        i++;
        setaddr=strtol(argv[i], NULL, 0);
        i++;
        setvalue=strtol(argv[i], NULL, 0);
        setlen=1;

        if (setlen>0 && init_radio(&serial, port, model, PMODE_PROGRAM)) {
           if (!config_valid)  {
              read_memory(serial, config, 1); // only read the config area
              config_valid=1;
              }

           memset(((void *)config + setaddr), setvalue & 0xFF, setlen & 0xFF);

           write_memory(serial, config, 1); // only write the config area
           }
        }
     else if (strcmp(argv[i],"-f")==0) { // fill config bytes
        i++;
        setaddr=strtol(argv[i], NULL, 0);
        i++;
        setvalue=strtol(argv[i], NULL, 0);
        i++;
        setlen=strtol(argv[i], NULL, 0);

        if (setlen>0 && init_radio(&serial, port, model, PMODE_PROGRAM)) {
           if (!config_valid) {
              read_memory(serial, config, 1); // only read the config area
              config_valid=1;
              }
              
           memset(((void *)config + setaddr), setvalue & 0xFF, setlen & 0xFF);

           write_memory(serial, config, 1); // only write the config area
           }
        }
     else if (strcmp(argv[i],"-D")==0) { // dump a section of memory
        i++;
        setaddr=strtol(argv[i], NULL, 0);
        i++;
        setlen=strtol(argv[i], NULL, 0);
        if (setlen>0 && init_radio(&serial, port, model, radio_mode == PMODE_NONE ? PMODE_PROGRAM : radio_mode)) {
           char *data=malloc(setlen + 16);
           char buf[12];
           int p=0;
           
           // read_block = 16bit address, read_block2 = 32bit address
           while (p<setlen) {
              if (read_block(serial, data+p, buf, setaddr+p, 8) ||
                  read_block(serial, data+p+8, buf, setaddr+p+8, 8)
                 ) {
                 hexdump(data+p, 16);
                 printf("\n");
                 p+=16;
                 }
              else {
                 p=setlen;
                 }
              }
           free(data);
           }
        }
     else if (strcmp(argv[i],"-e")==0) { // set expected response
        i++;
        cmd_expect=strtol(argv[i],NULL,0);
        }
     else if ((strcmp(argv[i],"-t")==0)  ||
              (strcmp(argv[i],"-T")==0) ||
              (strcmp(argv[i],"-F")==0)
             )
        {				// test commands
        int from_program_mode=0;
        int j;
        char *ep;
        if (strcmp(argv[i],"-T")==0) from_program_mode=PMODE_PROGRAM;
        if (strcmp(argv[i],"-F")==0) from_program_mode=PMODE_FIRMWARE;
        
        i++;
        cmd_len=strtol(argv[i],NULL,0);
        if (cmd_len) {
           printf("%d byte command\n", cmd_len);
           for (j=0; j<cmd_len; j++) {
               i++;
               ep=NULL;
               cmd_buf[j] = strtol(argv[i], &ep, 0);
               if (ep && !*ep) {
                  printf("  0x%02x", cmd_buf[j]);
                  }
               else {
                  if (*argv[i] == '\\') 
                       cmd_buf[j]=argv[i][1];
                  else cmd_buf[j]=argv[i][0];
                  printf(" %c (0x%02x)", cmd_buf[j], cmd_buf[j]);
                  }
               }
           printf("\n");
           }
        else {
           i++;
           printf("string command: '%s'\n", argv[i]);
           strcpy(cmd_buf, argv[i]);
           cmd_len=strlen(argv[i]);
           }
           
        if (cmd_len) {
           if (from_program_mode) {
              if (init_radio(&serial, port, model, from_program_mode)) {
                 test_cmd(serial, cmd_buf, cmd_len, cmd_expect);
                 }
              else {
                 printf("Unable to enter program mode\n");
                 }
              }
           else {
              if (serial<0) {
                 serial=openserial(port);
                 if (serial < 0) {
                    perror("Error opening port");
                    }
                 RX_on(serial);
                 }
              test_cmd(serial, cmd_buf, cmd_len, cmd_expect);
              }
           }
        }
     i++;
     }
     
#if 0
        // update a couple of channels - if you want to do more than a few
        // it's probably easier to read/write the whole memory (but slower).
        read_channel(serial, config, 1);
        read_channel(serial, config, 2);
        config->channels[1].tx_power=TXPOWER_HI;
        config->channels[2].tx_power=TXPOWER_HI;
        write_channel(serial, config, 1);
        write_channel(serial, config, 2);
#endif

  if (serial>0) {
     tcdrain(serial);
     usleep(200000);
     close(serial);
     serial=-1;
     }
}
