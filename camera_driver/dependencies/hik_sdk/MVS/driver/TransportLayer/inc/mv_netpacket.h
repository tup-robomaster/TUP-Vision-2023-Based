

#ifndef     __MV_NET_PACKET_H__
#define     __MV_NET_PACKET_H__


#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//#include "misc.h"
#include "mv_nettypes.h"

extern int NET_PacketDestroy(void* aPacket);
extern void* NET_PacketCopy(void* aPacket);
extern int NET_PacketCopyToUser(void* aPacket, unsigned int nOffset, void* pDst, unsigned int nLen);
extern NET_HEADER_IPV4* NET_ExtractIphdr(void* aPacket);
extern int  NET_ExtractIpLen(NET_HEADER_IPV4* aIpheader);
extern unsigned char* NET_ExtractSkbufData(void *aPacket);
extern unsigned short NET_Htons(unsigned short aAddr);
extern unsigned int NET_Htonl(unsigned int aAddr);
extern unsigned int NET_Ntohl(unsigned int aAddr);
extern unsigned short NET_Ntohs(unsigned short aAddr);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //__MV_NET_PACKET_H__

