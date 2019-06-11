#ifdef ESP8266//Se estiver usando ESP8266, automáticamente irá adicionar as bibliotecas do ESP8266.
#include <ESP8266WiFi.h>
#include <WiFiServer.h>
#elif defined ESP32//Se estiver usando ESP32, fara a mesma operaçao.
#include "WiFi.h"
#include "Arduino.h"
#include "Communication.h"
#endif
 
WiFiServer sv(555);//Cria o objeto servidor na porta 555
WiFiClient cl;//Cria o objeto cliente.
 
void Calling::Setting()
{
    Serial.begin(115200);//Habilita a comm serial.
 
    WiFi.mode(WIFI_AP);//Define o WiFi como Acess_Point.
    WiFi.softAP(ssid,password);//Cria a rede de Acess_Point.
 
    sv.begin();//Inicia o servidor TCP na porta declarada no começo.
}
void Calling::Looping()
{
    tcp();//Funçao que gerencia os pacotes e clientes TCP.
}
void Calling::tcp()
{
    if (cl.connected())//Detecta se há clientes conectados no servidor.
    {
        if (cl.available() > 0)//Verifica se o cliente conectado tem dados para serem lidos.
        {
            String req = "";
            while (cl.available() > 0)//Armazena cada Byte (letra/char) na String para formar a mensagem recebida.
            {
                char z = cl.read();
                req += z;
            }
            switch (req.charAt(0))
            {
                case 'S':
                {
                    aux = "";
                    int cont = req.length() - 1;
                    for(int i=1 ; i<=cont ; i++)
                    {
                        char y = req.charAt(i);
                        aux += y;
                    }
                    if(aux.equals("100"))
                    {
                        value += 32;
                        cl.print(" Offset: ");
                        cl.print(value);
                        cl.print("\n");
                    }
                    else if(aux.equals("200"))
                    {
                        value -= 32;
                        cl.print(" Offset: ");
                        cl.print(value);
                        cl.print("\n");
                    }
                    else if(aux.equals("100") == false || aux.equals("200") == false)
                    {
                        value = aux.toInt();
                        cl.print(" Offset: ");
                        cl.print(value);
                        cl.print("\n"); 
                    }                      
                }
                    break;
                case 'p':
                {
                    aux = "";
                    int cont_1 = req.length() - 1;
                    for(int i=1 ; i<=cont_1 ; i++)
                    {
                        char y = req.charAt(i);
                        aux += y;
                    }
                    if(aux.equals("100"))
                    {
                        KP += 0.2;
                        cl.print(" KP: ");
                        cl.print(KP,4);
                        cl.print("\n");
                    }
                    else if(aux.equals("200"))
                    {
                        KP -= 0.2;
                        cl.print(" KP: ");
                        cl.print(KP,4);
                        cl.print("\n");
                    }
                    else if(aux.equals("100") == false || aux.equals("200") == false)
                    {
                        KP = aux.toDouble();
                        cl.print(" KP: ");
                        cl.print(KP,4);
                        cl.print("\n"); 
                    }                    
                }
                    break;
                case 'i':
                {
                    aux = "";
                    int cont_2 = req.length() - 1;
                    for(int i=1 ; i<=cont_2 ; i++)
                    {
                        char y = req.charAt(i);
                        aux += y;
                    }
                    if(aux.equals("100"))
                    {
                        KI += 0.01;
                        cl.print(" KI: ");
                        cl.print(KI,4);
                        cl.print("\n"); 
                    }
                    else if(aux.equals("200"))
                    {
                        KI -= 0.01;
                        cl.print(" KI: ");
                        cl.print(KI,4);
                        cl.print("\n"); 
                    }
                    else if(aux.equals("100") == false || aux.equals("200") == false)
                    {
                        KI = aux.toDouble();
                        cl.print(" KI: ");
                        cl.print(KI,4);
                        cl.print("\n"); 
                    }
                }
                    break;  
                case 'd':
                {
                    aux = "";
                    int cont_3 = req.length() - 1;
                    for(int i=1 ; i<=cont_3 ; i++)
                    {
                        char y = req.charAt(i);
                        aux += y;
                    }
                    if(aux.equals("100"))
                    {
                        KD += 0.2;
                        cl.print(" KD: ");
                        cl.print(KD,4);
                        cl.print("\n"); 
                    }
                    else if(aux.equals("200"))
                    {
                        KD -= 0.2;
                        cl.print(" KD: ");
                        cl.print(KD,4);
                        cl.print("\n"); 
                    }
                    else if(aux.equals("100") == false || aux.equals("200") == false)
                    {
                        KD = aux.toDouble();
                        cl.print(" KD: ");
                        cl.print(KD,4);
                        cl.print("\n"); 
                    }
                }
                    break;	
            }
        }
    }
    else//Se nao houver cliente conectado,
    {
        cl = sv.available();//Disponabiliza o servidor para o cliente se conectar.
        //delay(1);
    }
    
}
void Calling::ReadWifi(uint32_t *going, double *Kp, double *Ki, double *Kd)
{
    *going = value;
    *Kp    = KP;
    *Ki    = KI;
    *Kd    = KD;
}
void Calling::WriteWifi(float angX_, float angY_, float angZ_,double _kp, double _ki,double _kd,int16_t pwm1, int16_t pwm2,int16_t pwm3, int16_t pwm4)
{
    if(millis() - time >= 2000)
    {
        cl.print("X: ");
        cl.print(angX_);
        cl.print(" Y: ");
        cl.print(angY_);
        cl.print(" Z: ");
        cl.print(angZ_);
        cl.print(" KP: ");
        cl.print(_kp);
        cl.print(" KI: ");
        cl.print(_ki);
        cl.print(" KD: ");
        cl.print(_kd);
        cl.print(" PWM1: ");
        cl.print(pwm1);
        cl.print(" PWM2: ");
        cl.print(pwm2);
        cl.print(" PWM3: ");
        cl.print(pwm3);
        cl.print(" PWM4: ");
        cl.print(pwm3);
        cl.print("\n");

        time = millis();
    }
}









 // //Envia uma resposta para o cliente
        //     cl.print("\nO servidor recebeu sua mensagem");
        //     cl.print("\n...Seu IP: ");
        //     cl.print(cl.remoteIP());
        //     cl.print("\n...IP do Servidor: ");
        //     cl.print(WiFi.softAPIP());
        //     Serial.print("\n...Tamanho da mensagem: ");
        //     Serial.print(Size);
        //     Serial.print("\n...Valor da mensagem: ");
        //     Serial.print(Valor);
        //     cl.print("\n...Sua mensagem: " + req + "\n");