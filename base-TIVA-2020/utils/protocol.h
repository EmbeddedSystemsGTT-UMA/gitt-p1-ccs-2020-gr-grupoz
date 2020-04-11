/* protocol.h
 *
 * Este fichero define las funciones para implementar la comunicacion mediante
 * el protocolo propuesto
 *
 * Los mensajes que se envian por el perfil USB constan de un byte de INICIO,
 * un byte de comando, un campo de datos opcional de longitud variable que depende del comando,
 * un CRC de 2 bytes y un byte de FIN.
 *
 */


#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include<stdlib.h>
#include<string.h>
#include<FreeRTOS.h>

//Caracteres especiales
#define START_FRAME_CHAR 0xCF
#define STOP_FRAME_CHAR 0xDF
#define ESCAPE_CHAR 0xEF
#define STUFFING_MASK 0x02

//Definiciones generales (se podrian usar tambien los stdtypes)
#define INT32U unsigned int
#define INT16U unsigned short
#define INT8U unsigned char

#define CHEKSUM_TYPE INT16U
#define COMMAND_TYPE INT8U

#define CHECKSUM_SIZE (sizeof(CHEKSUM_TYPE))
#define COMMAND_SIZE (sizeof(COMMAND_TYPE))
#define START_SIZE (1)
#define END_SIZE (1)

#define MINIMUN_FRAME_SIZE (START_SIZE+COMMAND_SIZE+CHECKSUM_SIZE+END_SIZE)

#define MAX_DATA_SIZE (32)
#define MAX_FRAME_SIZE (2*(MAX_DATA_SIZE))

//Codigos de Error
#define PROT_ERROR_BAD_CHECKSUM (-1)
#define PROT_ERROR_RX_FRAME_TOO_LONG (-2)
#define PROT_ERROR_NOMEM (-3)
#define PROT_ERROR_STUFFED_FRAME_TOO_LONG (-4)
#define PROT_ERROR_COMMAND_TOO_LONG (-5)

//Macros para obtener campos
#define NetToU32(array,posicion) ((((INT32U)array[posicion+3])<<24)|(((INT32U)array[posicion+2])<<16)|(((INT32U)array[posicion+1])<<8)|((INT32U)array[posicion]))
#define NetToU16(array,posicion) ((((INT16U)array[posicion+1])<<8)|(((INT16U)array[posicion])))
#define NetToU8(array,posicion) ((INT8U)array[posicion])

#define SwapBytes16(datos) ((0xFF&(((unsigned short)datos)>>8))|((((unsigned short)datos)&0xFF)<<8))                  

//Macros que obtienen campos del paquete
#define decode_command_type(paquete,offset) (NetToU8(paquete,(offset)))
#define extract_packet_command_param(paquete,payload,campo) memcpy(campo,paquete+COMMAND_SIZE,payload);
#define get_packet_command_param_pointer(paquete) (paquete+COMMAND_SIZE);

//Otra macro que puede ser de utilidad
#define check_command_param_size(frame_size,param_size) (!(frame_size-param_size-COMMAND_SIZE-CHECKSUM_SIZE))
#define get_param_size(frame_size) (frame_size-COMMAND_SIZE-CHECKSUM_SIZE)

//Funciones de la libreria
int32_t create_frame(uint8_t *frame, uint8_t command_type, void * param, int32_t param_size, int32_t max_size);
int32_t send_frame(uint8_t *frame, int32_t FrameSize);
int32_t receive_frame(uint8_t *frame, int32_t maxFrameSize);
int32_t destuff_and_check_checksum(uint8_t *frame, int32_t max_size);

//Codigos de los comandos. EL estudiante deberá definir los códigos para los comandos que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt

typedef enum {
	COMANDO_NO_IMPLEMENTADO,
	COMANDO_PING,
	//etc, etc...
} commandTypes;

//Estructuras relacionadas con los parametros de los comandos. El estuadiante debera crear las
// estructuras adecuadas a los comandos usados, y asegurarse de su compatibilidad con el extremo Qt

//#pragma pack(1)	//Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
//Con lo de abajo consigo que el alineamiento de las estructuras en memoria del microcontrolador no tenga relleno
#define PACKED __attribute__ ((packed))
		 
typedef struct {
	uint8_t command;
} PACKED PARAM_COMANDO_NO_IMPLEMENTADO;

//#pragma pack()	//...Pero solo para los comandos que voy a intercambiar, no para el resto


#endif
