/*
 * Listado de los tipos de comandos empleados en la aplicación, así como definiciones de sus parámetros.
 * (TODO) Incluir aquí las respuestas a los comandos?
*/
#ifndef __USB_COMMANDS_TABLE_H
#define __USB_COMMANDS_TABLE_H

#include<stdint.h>

//Codigos de los comandos. EL estudiante deberá definir los códigos para los comandos que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt

typedef enum {
    COMANDO_NO_IMPLEMENTADO,
    COMANDO_PING,
    //etc, etc...
} commandTypes;

//Estructuras relacionadas con los parametros de los comandos. El estuadiante debera crear las
// estructuras adecuadas a los comandos usados, y asegurarse de su compatibilidad con el extremo Qt

//#pragma pack(1)   //Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
//Con lo de abajo consigo que el alineamiento de las estructuras en memoria del microcontrolador no tenga relleno
#define PACKED __attribute__ ((packed))

typedef struct {
    uint8_t command;
} PACKED PARAM_COMANDO_NO_IMPLEMENTADO;

//#pragma pack()    //...Pero solo para los comandos que voy a intercambiar, no para el resto




#endif
