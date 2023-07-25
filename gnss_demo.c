#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <string.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include "ublox_cfg.h"
#include "gnss.h"

/*Parse GNSS*/
#define  NMEA_MAX_SIZE  255
typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    int     sv_flag;
    GnssLocation fix;
    GnssSvStatus svs;
    char    in[ NMEA_MAX_SIZE+1 ];
} NmeaReader;


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/
typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  32

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

struct broadcast_node {
    char ip_addr[32];
    int port;
};

struct broadcast_node broadcast_list[] = {
    {"127.0.0.1", 40400},
    {"127.0.0.1", 40401},
};

extern int errno;
/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;
    char  temp[16];

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    // the initial '$' is optional
    if (p < end && (p[0] == '$' || p[0] == '#'))
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;

        if (q >= p) {
            if (count < MAX_NMEA_TOKENS) {
                t->tokens[count].p   = p;
                t->tokens[count].end = q;
                count += 1;
            }
        }
        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}

static void
nmea_reader_update_utc_diff( NmeaReader*  r )
{
#if 0
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));

    r->utc_diff = time_utc - time_local;
#else
    r->utc_diff = 8*60*60;
#endif
}

static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        msec;
    struct tm  tm;
    time_t     fix_time;

    if (tok.p + 8 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        // no date yet, get current one
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        r->utc_year = tm.tm_year + 1900;
        r->utc_mon  = tm.tm_mon + 1;
        r->utc_day  = tm.tm_mday;
    }

    msec    = str2int(tok.p+7, tok.p+8);

    tm.tm_hour  = str2int(tok.p,   tok.p+2);
    tm.tm_min   = str2int(tok.p+2, tok.p+4);
    tm.tm_sec   = str2int(tok.p+4, tok.p+6);
    tm.tm_year  = r->utc_year - 1900;
    tm.tm_mon   = r->utc_mon - 1;
    tm.tm_mday  = r->utc_day;
    tm.tm_isdst = -1;

    fix_time = mktime( &tm ) + r->utc_diff;
    /* unit: mseconds */
    r->fix.timestamp = ((long long)fix_time * 1000l) + ((long long)msec * 100l);
    return 0;
}

static int
nmea_reader_update_time_s( NmeaReader*  r, Token  tok )
{
    if (tok.p >= tok.end)
        return -1;

    r->fix.timestamp  = (long long)(str2float(tok.p, tok.end))*1000;
    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end) {
        return -1;
    }
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;

    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, time );
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GNSS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_latlong_s( NmeaReader* r, Token latitude, Token longitude )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p > tok.end) {
        return -1;
    }
    lat = str2float(tok.p, tok.end);

    tok = longitude;
    if (tok.p > tok.end) {
        return -1;
    }
    lon = str2float(tok.p, tok.end);

    r->fix.flags    |= GNSS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GNSS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GNSS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}

#define KN2MS(x) ((x) * 1852.0 / 3600.0)
static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GNSS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end);
    r->fix.speed    = KN2MS(r->fix.speed);
    return 0;
}

static int
nmea_reader_update_accuracy( NmeaReader*  r,
                          Token        accuracy )
{
#if 0
    // Always return 20m accuracy.
    // Possibly parse it from the NMEA sentence in the future.
    r->fix.flags    |= GNSS_LOCATION_HAS_ACCURACY;
    r->fix.accuracy = 20;
    return 0;
#else
    Token   tok = accuracy;
    
    if (tok.p >= tok.end)
        return -1;

    r->fix.flags    |= GNSS_LOCATION_HAS_ACCURACY;
    r->fix.accuracy  = str2float(tok.p, tok.end);
    return 0;
#endif
}


static void
nmea_reader_parse( NmeaReader*  r )
{
    static int s_sv_num = -2;
    int sv_num;
   /* we received a complete sentence, now parse it to generate
    * a new GNSS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;
    
    if (r->pos < 9) {
        return;
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);

    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end) {
        return;
    }

    // ignore first two characters.
    tok.p += 2;
    if ( !memcmp(tok.p, "STPOSA", 6) ) {
        Token  tok_time          = nmea_tokenizer_get(tzer,6);
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,10);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,11);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,12);
        Token  tok_svn           = nmea_tokenizer_get(tzer,22);

        if ( memcmp(tok_fixStatus.p, "NONE", 4) )
        {
            r->sv_flag = 1;
            nmea_reader_update_time_s( r, tok_time );

            nmea_reader_update_latlong_s( r, tok_latitude, tok_longitude );

            sv_num = str2int(tok_svn.p, tok_svn.end);
            sv_num /= 6;
            if (s_sv_num != sv_num) {
                s_sv_num = sv_num;
            }
        }
        else
        {
            r->sv_flag = 0;
            if (s_sv_num != -1) {
                s_sv_num = -1;
            }
        }

    } else if ( !memcmp(tok.p, "GGA", 3) ) {
        // GNSS fix
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
        Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
        Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

        nmea_reader_update_time(r, tok_time);
        nmea_reader_update_latlong(r, tok_latitude,
                                      tok_latitudeHemi.p[0],
                                      tok_longitude,
                                      tok_longitudeHemi.p[0]);
        nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);

    } else if ( !memcmp(tok.p, "GSA", 3) ) {
        Token tok_fixStatus = nmea_tokenizer_get(tzer,2);
        if (tok_fixStatus.p != tok_fixStatus.end && (tok_fixStatus.p[0]=='2' || tok_fixStatus.p[0]=='3'))
        {
            r->sv_flag = 1;
            Token tok_accuracy = nmea_tokenizer_get(tzer,15);
            nmea_reader_update_accuracy(r, tok_accuracy);
            int i;
            for (i=3; i<=14; i++)
            {
                Token tok_prn = nmea_tokenizer_get(tzer,i);
                int n_prn = str2int(tok_prn.p, tok_prn.end);
                if (n_prn>0)
                {
                    r->svs.used_in_fix_mask |= (1<<(n_prn-1));
                }
            }
        }
        else
        {
            r->sv_flag = 0;
            if (s_sv_num != -1) {
                s_sv_num = -1;
            }
        }
    } else if ( !memcmp(tok.p, "GSV", 3) && *(tok.p-1)=='P' ) {
        Token tok_noSatellites = nmea_tokenizer_get(tzer, 3);
        int noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);
        Token tok_noSentences = nmea_tokenizer_get(tzer, 1);
        Token tok_sentence = nmea_tokenizer_get(tzer, 2);
        int sentence = str2int(tok_sentence.p, tok_sentence.end);
        int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
        int i,ns;

        if (1 == sentence)
        {
            r->svs.num_svs = noSatellites;
            r->svs.ephemeris_mask = 0;
            r->svs.almanac_mask = 0;

            sv_num = noSatellites;
            sv_num /= 6;
            if (s_sv_num != sv_num && r->sv_flag != 0) {
                s_sv_num = sv_num;
            }
        }

        for (i=0; i<4; i++)
        {
            ns = (sentence-1)*4+i;
            if (ns >= noSatellites)
                break;
            Token tok_prn = nmea_tokenizer_get(tzer, i*4+4);
            Token tok_elevation = nmea_tokenizer_get(tzer, i*4+5);
            Token tok_azimuth = nmea_tokenizer_get(tzer, i*4+6);
            Token tok_snr = nmea_tokenizer_get(tzer, i*4+7);
            r->svs.sv_list[ns].prn = str2int(tok_prn.p, tok_prn.end);
            r->svs.sv_list[ns].elevation = str2float(tok_elevation.p, tok_elevation.end);
            r->svs.sv_list[ns].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
            r->svs.sv_list[ns].snr = str2float(tok_snr.p, tok_snr.end);
            if (r->svs.sv_list[ns].prn > 0) {
                r->svs.ephemeris_mask |= (1<<(r->svs.sv_list[ns].prn-1));
                r->svs.almanac_mask |= (1<<(r->svs.sv_list[ns].prn-1));
            }
        }
    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
        Token  tok_speed         = nmea_tokenizer_get(tzer,7);
        Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
        Token  tok_date          = nmea_tokenizer_get(tzer,9);
        
        r->svs.used_in_fix_mask = 0;

        if (tok_fixStatus.p[0] == 'A')
        {
            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }

    } else if ( !memcmp(tok.p, "VTG", 3) ) {
        Token  tok_bearing       = nmea_tokenizer_get(tzer,1);
        Token  tok_speed         = nmea_tokenizer_get(tzer,7);

        nmea_reader_update_bearing( r, tok_bearing );
        nmea_reader_update_speed  ( r, tok_speed );

    } else {
        tok.p -= 2;
    }
}

static void
nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
    r->fix.size = sizeof(r->fix);
    r->sv_flag = 0;
    r->svs.size = sizeof(r->svs);

    nmea_reader_update_utc_diff( r );
}

static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        nmea_reader_parse( r );
        r->pos = 0;
    }
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       G P S   C O N F I G                             *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       G P S   I N T E R F A C E                       *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static int gnss_set_chk(unsigned char *msg, int size)
{
    int i = 0;

    if((0 == msg) || (size <= 0))
    {
        return -1;
    }

    for(i=0; i<(size-4); i++)
    {
        msg[size-2] += msg[i+2];
        msg[size-1] += msg[size-2];
    }

    return 0;
}

static int gnss_write_tty(int fd, unsigned char *buf, int size)
{
    int cnt = 0;
    int sum = 0;

    if((0 == buf) || (size <= 0) || (fd<0))
    {
        return -1;
    }

    while (sum < size)
    {
        cnt = write(fd, (buf+sum), (size-sum));
        if(cnt <= 0)
        {
            return -1;
        }

        sum += cnt;
    }

    return 0;
}

static int tty_open(const char *dev)
{
    int flags = 0;
    flags = O_RDWR | O_NOCTTY;
    return open(dev, flags);
}

static int tty_init(const char *dev, speed_t speed)
{
    int fd;
    int ret;
    struct termios old_tty_opt;
    struct termios new_tty_opt;

    fd = tty_open(dev);
    if (fd < 0) {
        fprintf(stderr, "open %s, error %s...\n", dev, strerror(errno));
        return -1;
    }

    memset(&old_tty_opt, 0, sizeof(old_tty_opt));
    memset(&new_tty_opt, 0, sizeof(new_tty_opt));
    ret = tcgetattr(fd, &old_tty_opt);
    if (ret < 0) {
        close(fd);
        fprintf(stderr, "tcgetattr error %s...\n", strerror(errno));
        return -1;
    }
    new_tty_opt = old_tty_opt;
    cfsetispeed(&new_tty_opt, speed);
    cfsetospeed(&new_tty_opt, speed);
    new_tty_opt.c_iflag &= ~(IXON | IXOFF);
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tty_opt.c_oflag &= ~OPOST;
    tcflush(fd, TCIOFLUSH);
    
    ret = tcsetattr(fd, TCSANOW, &new_tty_opt);
    if (ret < 0) {
        close(fd);
        fprintf(stderr, "tcsetattr error %s...\n", strerror(errno));
        return -1;
    }
    close(fd);
    return 0;
}

static int gnss_receiver_init(const char *dev, unsigned char **msg, int *msg_len, int count)
{
    int i = 0;
    int retry = 3;
    int fd;

    fd = tty_open(dev);
    if (fd < 0) {
        fprintf(stderr, "open %s, error %s...\n", dev, strerror(errno));
        return -1;
    }

    for (i = 0;i < count;i++){
        gnss_set_chk(msg[i], msg_len[i]);
        gnss_write_tty(fd, msg[i], msg_len[i]);
    }

    close(fd);
    return 0;
}

static int gnss_broadcast_init()
{
    int fd = -1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    return fd;
}

static int gnss_broadcast(int fd, unsigned char *data, int len)
{
    struct sockaddr_in peer_gnss_sockaddr;
    int i, count;
    int r;

    count = sizeof(broadcast_list)/sizeof(struct broadcast_node);
    for (i = 0;i < count;i++) {
        memset((void*)(&peer_gnss_sockaddr), 0, sizeof(peer_gnss_sockaddr));
        peer_gnss_sockaddr.sin_family = AF_INET;
        peer_gnss_sockaddr.sin_addr.s_addr = inet_addr(broadcast_list[i].ip_addr);
        peer_gnss_sockaddr.sin_port = htons(broadcast_list[i].port);
        r = sendto(fd, data, len, 0, (struct sockaddr*)&peer_gnss_sockaddr, sizeof(peer_gnss_sockaddr));
        if (r != len)
            break;
    }

    return r;
}

#define TTYDEV  "/dev/ttyS1"

static unsigned char gnss_tty_buf[2048] = {0};

/*
 1. configure AP's uart
 2. configure GPS receiver
 3. receive GPS receiver message
 4. parse the NMEA message
 5. send the GNSS data to other tasks
 */
int main(int argc, char **argv)
{
    int broadcast_fd = -1;
    int nn;
    int r = -1;
    NmeaReader  reader[1];
    fd_set read_set;
    struct timeval tv_timeout;
    int max_fd;
    int tty_fd;

    tty_init(TTYDEV, B9600);
    gnss_receiver_init(TTYDEV,
                       gnss_receiver_cfg_message,
                       gnss_receiver_cfg_message_len,
                       cfg_message_cnt
                       );
    tty_init(TTYDEV, B115200);

    broadcast_fd = gnss_broadcast_init();
    nmea_reader_init(reader);
    tty_fd = tty_open(TTYDEV);
    while (1) {
        FD_ZERO(&read_set);
        FD_SET(tty_fd, &read_set);
        max_fd = tty_fd + 1;
        tv_timeout.tv_sec = 20;
        tv_timeout.tv_usec = 0;
        r = select(max_fd, &read_set, NULL, NULL, &tv_timeout);
        if (r > 0) {
            r = read(tty_fd, gnss_tty_buf, sizeof(gnss_tty_buf));
            if (r <= 0) {
                continue;
            }
            for (nn = 0; nn < r; nn++)
                nmea_reader_addc(reader, gnss_tty_buf[nn]);
            r = gnss_broadcast(broadcast_fd, &(reader[0].fix), sizeof(GnssLocation));
        } else if (r == 0) {
            /* timeout */
            continue;
        } else {
            /* error */
            continue;
        }
    }

    exit(0);
}
