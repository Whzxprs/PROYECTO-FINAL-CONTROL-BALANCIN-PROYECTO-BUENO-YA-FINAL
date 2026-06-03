#include <iostream>
#include <string.h>
#include<dos.h>
#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <conio.h>
#include <stdlib.h>
#include <stdbool.h>

#define COM_PORT "COM8"

//Parámetros
#define pi_     3.141593
#define pi_s2   1.570796
#define Ts      0.01
#define R	    0.034
#define omegaM	20.0
#define alphaM	1.4
#define tauM	0.3	//par máximo en rueda [Nm]
//#define omegaM	44.0
#define uM		10.5

unsigned char flagcom=0,flagfile=0,vdg,vg,thetag,alphag,omegalg,omegarg,ulg,urg;
// Variables auxiliares
float t=0,omegar=0,omegal=0;
float ur=0,ul=0;
float v=0,theta=0;
float alpha=0,vd; 
float mv=(2*omegaM*R)/255.0,mtheta=(2*0.3)/255.0,malpha=(2*alphaM)/255.0,momega=(2*omegaM)/255.0,mu=(2*uM)/255.0;

using namespace std;
int main()
{
    HANDLE h; /*handler, sera el descriptor del puerto*/
    DCB dcb; /*estructura de configuracion*/
    DWORD dwEventMask; /*mascara de eventos*/
    FILE *fp;
    
    if((fp=fopen("datos.txt","w+"))==NULL)
	{
	  printf("No se puede abrir el archivo.\n");
	  exit(1);
	}    
        
        
    /*abrimos el puerto*/
    h=CreateFile(COM_PORT,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    
    if(h == INVALID_HANDLE_VALUE) 
	{
         /*ocurrio un error al intentar abrir el puerto*/
    }
             
	/*obtenemos la configuracion actual*/
	if(!GetCommState(h, &dcb)) 
	{
         /*error: no se puede obtener la configuracion*/
        printf("No se puede abrir");
    }
         
    /*Configuramos el puerto*/
	dcb.BaudRate = 115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = TRUE;
         
    /* Establecemos la nueva configuracion */
    if(!SetCommState(h, &dcb)) 
	{
        /* Error al configurar el puerto */
    }
                  
    DWORD n;
    char enviar;
    int recibido;
    
				             
    /* Para que WaitCommEvent espere el evento RXCHAR */
    SetCommMask(h, EV_RXCHAR);
    while(1) 
	{	
		
    	recibido=0;
        while(1) 
		{
        	ReadFile(h, &recibido, 1/* leemos un byte */, &n, NULL);
            if(!n)
            	break;
            else
			{
            	if(flagcom!=0)
				flagcom++;
				if((recibido==0xAA)&&(flagcom==0))
   				{
        			flagcom=1;
   				}
				if(flagcom==2)
   				{
        			vdg = recibido;
				}
   				
   				if(flagcom==3)
   				{
        			vg = recibido;
   				} 
   				if(flagcom==4)
   				{
        			thetag = recibido;
   				} 
   				if(flagcom==5)
   				{
				   alphag = recibido;
   				} 
   				if(flagcom==6)
   				{
        			omegalg = recibido;
   				} 		
   				if(flagcom==7)
   				{
        			omegarg = recibido;
   				} 	   				
   				if(flagcom==8)
   				{
        			ulg =recibido;
   				} 	   				
   				if(flagcom==9)
   				{
					urg = recibido;
					
					vd=(vdg-127)*mv;
					v=(vg-127)*mv;
					theta=(thetag-127)*mtheta;
					alpha=(alphag-127)*malpha;
					omegal=(omegalg-127)*momega;
					omegar=(omegarg-127)*momega;
					ul=(ulg-127)*mu;
					ur=(urg-127)*mu;
					
	                //Imprimiendo en pantalla
	                //printf("%i\t\n", pos);
					printf("%.2f\t%.2f\t%.2f\n",t,alpha,theta);
	    			//fprintf(fp,"%i\t\n",  pos);
					/*escribir algunos datos en el archivo*/
	    			fprintf(fp,"%.2f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",t,alpha,theta,v,vd,omegal,omegar,ul,ur);
	    			t=t+Ts;//t+=Ts;
	    			flagcom=0;
					
				}				
   			}//cierre del else       
            //printf("%c",recibido);
	    	//cout << recibido; /* mostramos en pantalla */
    	}//CIERRE WHILE()
    }//CIERRE WHILE()
fclose(fp);             
return 0;
}
