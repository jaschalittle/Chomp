int COSMOS_connect(const char *host, const char *port);
int COSMOS_readpkt(int fd, struct timeval *stamp, char **target, char **packet, uint32_t *datalen, uint8_t **data);

