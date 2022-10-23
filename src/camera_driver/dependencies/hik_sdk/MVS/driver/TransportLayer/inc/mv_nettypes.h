
#ifndef __MV_NETTYPE_H__
#define __MV_NETTYPE_H__


#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//#include "misc.h"


#define NET_PROTOCOL_UDP                17


#pragma pack( push, 1 )
// ipv4 header
typedef struct
{
    // 版本号(Version)4bit, 报头长度(Internet Header Length，IHL)4bit
    unsigned char       		VersionAndHeaderLength;

    // 服务类型 (Type of Service ，TOS) 8bit
    unsigned char       		DifferentiatedServices; 

    // 总长度字段 16bit
    unsigned short     			TotalLength;
    
    // 标志字段: 16bit
    unsigned short      		Identification;

    // 标志位字段: 3bit, 段偏移字段：13bit
    unsigned short      		FlagsAndFragmentOffset;
    
    // 生存期（TTL: Time to Live）字段：8bit
    unsigned char       		TTL;

    // 协议字段: 8bit
    unsigned char       		Protocol;

    // 头部校验和字段: 16bit
    unsigned short      		HeaderChecksum ;
    
    // 源IP地址、目标IP地址字段: 各32bit
    unsigned int				SourceIP;
    unsigned int	       		DestinationIP;

}NET_HEADER_IPV4;
#pragma pack( pop )


#pragma pack( push, 1 )
// udp header
typedef struct
{
    unsigned short 				SourcePort;
    unsigned short 				DestinationPort;
    unsigned short 				Length;
    unsigned short 				Checksum;
}NET_HEADER_UDP;
#pragma pack( pop )


#ifdef __cplusplus
}
#endif // __cplusplus


#endif	// __MV_NETTYPE_H__
