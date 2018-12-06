#include "nxbus_engine.h"
#include <hiredis.h>

static redisContext *c;
static redisReply *reply;

void nxbus_init(void) {
    unsigned int j;

    const char *hostname = "127.0.0.1";
    int port = 6379;

    struct timeval timeout = { 1, 500000 }; // 1.5 seconds
    c = redisConnectWithTimeout(hostname, port, timeout);
    if (c == NULL || c->err) {
        if (c) {
            printf("Connection error: %s\n", c->errstr);
            redisFree(c);
        } else {
            printf("Connection error: can't allocate redis context\n");
        }
        exit(1);
    }
    /* PING server */
    reply = redisCommand(c,"PING");
    printf("PING: %s\n", reply->str);
    freeReplyObject(reply);
    return 0;
}

void nxbus_deinit(void) {
    /* Disconnects and frees the context */
    redisFree(c);
}

int nxbus_mset(ENUM_NXBUS_DATA_T data_type, const char *key_name, size_t dimension, ...) {
    int idx = 0;
    int rc = 0;
    double *vector_double;
    uint32_t *vector_uint32;
    va_list ap;
    reply = redisCommand(c,"MULTI");
    freeReplyObject(reply);
    va_start(ap, dimension);
    switch(data_type) {
        case NXBUS_DOUBLE:
            vector_double = va_arg(ap, double*);
            for (idx = 0; idx < dimension; ++idx) {
                reply = redisCommand(c,"SET %s[%d] %f ", key_name, idx, vector_double[idx]);
                freeReplyObject(reply);
            }
            break;
        case NXBUS_UINT32:
            vector_uint32 = va_arg(ap, uint32_t*);
            for (idx = 0; idx < dimension; ++idx) {
                reply = redisCommand(c,"SET %s[%d] %u ", key_name, idx, vector_uint32[idx]);
                freeReplyObject(reply);
            }
            break;
        default:
            fprintf(stderr, "[%s:%d]NXBUS Error: Data Type not found !\n", __FUNCTION__, __LINE__);
            rc |= 0x1;
            break;
    }
    va_end(ap);
    reply = redisCommand(c,"EXEC");
    for (idx = 0; idx < dimension; ++idx) {
        if(reply->element[idx]->type == REDIS_REPLY_ERROR) {
            printf("EXEC: %s\n", reply->str);
            rc |= 0x1;
        }
    }
    freeReplyObject(reply);
    return rc;
}

int nxbus_mget(ENUM_NXBUS_DATA_T data_type, const char *key_name, size_t dimension, ...) {
    int idx = 0;
    int rc = 0;
    double *vector_double = NULL;
    uint32_t *vector_uint32 = NULL;
    va_list ap;
    reply = redisCommand(c,"MULTI");
    freeReplyObject(reply);
    for (idx = 0; idx < dimension; ++idx) {
        reply = redisCommand(c,"GET %s[%d]", key_name, idx);
        freeReplyObject(reply);
    }

    reply = redisCommand(c,"EXEC");
    for (idx = 0; idx < dimension; ++idx) {
        if(reply->element[idx]->type == REDIS_REPLY_ERROR) {
            printf("[%d] EXEC: %s\n", idx, reply->str);
            rc |= 0x1;
        }
    }
    if(rc != 0)
        return rc;
    va_start(ap, dimension);
    switch(data_type) {
        case NXBUS_DOUBLE:
            vector_double = va_arg(ap, double*);
            for (idx = 0; idx < dimension; ++idx)
                sscanf(reply->element[idx]->str, "%lf", &vector_double[idx]);
            break;
        case NXBUS_UINT32:
            vector_uint32 = va_arg(ap, uint32_t*);
            for (idx = 0; idx < dimension; ++idx)
                sscanf(reply->element[idx]->str, "%u", &vector_uint32[idx]);
            break;
        default:
            fprintf(stderr, "[%s:%d]NXBUS Error: Data Type not found !\n", __FUNCTION__, __LINE__);
            rc |= 0x1;
            break;
    }
    va_end(ap);
    freeReplyObject(reply);

    return rc;
}
