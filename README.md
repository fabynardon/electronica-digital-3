# electronica-digital-3
Proyecto final para electronica digital 3

Este proyecto es una implementacion multicore que controla los parametros de un controlador PID (cmsis). En sintesis la aplicacion es la siguiente:

El cortex M0 recibe por via UART los valores del PID y se los envia al cortex M4 por medio del IPC

El cortex M4 utiliza un timer para disparar el ADC0 a una frecuencia determinada, al activarse la interrupcion se realiza los calculos del lazo de control con una planta emulada.

los datos del adc, el error, la entrada de la planta y la salida se envian mediante uart a un instrumento virtual hecho en labview desde donte tambien se envian las constantes del controlador (todo realizado en punto flotante).

(PARA PODER COMPILAR ESTE PROYECTO, TAMBIEN ES NECESARIO DESCARGAR EL REPOSITORIO LLAMADO EDU-CIAA-NXP)
