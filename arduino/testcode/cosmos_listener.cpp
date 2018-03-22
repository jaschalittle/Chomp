#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>

int COSMOS_connect(const char *host, const char *port)
{
    struct addrinfo hints;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    hints.ai_protocol = 0;
    struct addrinfo *result;
    int err;
    err = getaddrinfo(host, port, &hints, &result);
    if(err != 0)
    {
        std::cerr << "getaddrinfo: " << gai_strerror(err) << std::endl;
        return err;
    }

    int fd;
    struct addrinfo *rp;
    for(rp = result; rp != NULL; rp = rp->ai_next)
    {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if(fd == -1)
        {
            perror("socket");
            continue;
        }
        err = connect(fd, rp->ai_addr, rp->ai_addrlen);
        if(err == 0)
            break;
        close(fd);
    }
    freeaddrinfo(result);
    if(rp == NULL)
    {
        return -1;
    }
    return fd;
}

static int read_timestamp(int fd, struct timeval *stamp)
{
    struct {
        uint32_t sec, usec;
    } sstamp;
    int err;
    err = read(fd, (void *)&sstamp, sizeof(sstamp));
    stamp->tv_sec = __builtin_bswap32(sstamp.sec);
    stamp->tv_usec = __builtin_bswap32(sstamp.usec);
    return err;
}

static int read_pstring(int fd, char **st)
{
    uint8_t len;
    int err;
    err = read(fd, &len, 1);
    if(err<0) return err;
    *st = (char *)realloc(*st, len+1);
    err = read(fd, *st, len);
    (*st)[len] = '\x00';
    return err;
}

static int read_packet_data(int fd, uint32_t *datalen, uint8_t **data)
{
    int err;
    err = read(fd, datalen, sizeof(uint32_t));
    if(err<0) return err;
    *datalen = __builtin_bswap32(*datalen);
    assert(*datalen<1024);
    *data = (uint8_t *)realloc(*data, (size_t)*datalen);
    return read(fd, *data, *datalen);
}

int COSMOS_readpkt(int fd, struct timeval *stamp, char **target, char **packet, uint32_t *datalen, uint8_t **data)
{
    int err;
    err = read_timestamp(fd, stamp);
    if(err<0)
    {
        perror("unable to read timestamp");
        return err;
    }
    err = read_pstring(fd, target);
    if(err<0)
    {
        perror("unable to read target");
        return err;
    }
    err = read_pstring(fd, packet);
    if(err<0)
    {
        if(target) free(target);
        perror("unable to read packet");
        return err;
    }
    err = read_packet_data(fd, datalen, data);
    if(err<0)
    {
        if(*target) free(*target);
        if(*packet) free(*packet);
        perror("unable to read data");
    }
    return err;
}
