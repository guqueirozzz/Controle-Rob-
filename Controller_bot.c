/*
 * File: Controller_bot.c     
 * Date: 21/11/2020
 * Description: Projeto Controle Robô CC7711 
 * Author: Gustavo Queiroz 
 * Modifications: Melhoria nos sensores, identificação de colisões entre objetos móveis, não móveis e melhoria no acionamento dos leds.
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>


#define TIME_STEP 64
#define QtddSensoresProx 8
#define QtddLeds 10
#define dife 0.01

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  
  int i=0;
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx]; // Vetor de Sensores de proximidade do robo
  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0; // aceleradores do robô
  const double *posicao; //variável que recebe a posição do robo e ajuda printando os dados no console, para melhor entendimento do robo no mundo 3D.
  double x[500]; // vetor que armazenas a posição do X quando ocorre colisão
  double z[500]; // vetor que armazenas a posição do Z quando ocorre colisão
  double eixoP; //variavel que faz a comparação de valores positivos de x no mundo
  double eixoN; //variavel que faz a comparação de valores negativos de x no mundo
  int aux = 0;  // variável auxiliar que controla o armazenamento nos vetores x[aux] e z[aux] (while) 
  int contador = 0;  // contador para aumentar o tempo de colisão caso um objeto seja detectado
  int cont_caixa_movel = 0; // variavel de controle para identificar quantas vezes a caixa móvel foi colidida com o robô
  int cont_led = 0;  //contador para decidir quando piscar o led 
  int cont_acende_led = 0;   //flag que decide acender um led especifico a cada colisao
  
  
  /* necessary to initialize webots stuff */
  
  for(i=0;i<257;i++) texto[i]='0';
  
  wb_robot_init();
  
  //configurei MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  
  wb_motor_set_velocity(MotorEsquerdo,0); // inicialmente os valores setados para velcoidade do motor esquerdo é 0.
  wb_motor_set_velocity(MotorDireito,0); //  inicialmente os valores setados para velcoidade do motor direito é 0.
  
  
   //setando Sensores de Proximidade
   WbDeviceTag SensorProx[QtddSensoresProx];
   
   // Recebendo os sensores de proximidade do robô e-puck
   SensorProx[0] = wb_robot_get_device("ps0");
   SensorProx[1] = wb_robot_get_device("ps1");
   SensorProx[2] = wb_robot_get_device("ps2");
   SensorProx[3] = wb_robot_get_device("ps3");
   SensorProx[4] = wb_robot_get_device("ps4");
   SensorProx[5] = wb_robot_get_device("ps5");
   SensorProx[6] = wb_robot_get_device("ps6");
   SensorProx[7] = wb_robot_get_device("ps7");
   
   wb_distance_sensor_enable(SensorProx[0],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[1],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[2],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[3],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[4],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[5],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[6],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[7],TIME_STEP);
  
   WbNodeRef robot_node = wb_supervisor_node_get_from_def("dev_robot"); //busca o supervisor configurado (dev_robot foi o nome que dei)
   WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation"); //identifica o campo de posição
   
    //config leds
    WbDeviceTag Leds[QtddLeds];
    Leds[0] = wb_robot_get_device("led0");
    Leds[1] = wb_robot_get_device("led1");
    Leds[2] = wb_robot_get_device("led2");
    Leds[3] = wb_robot_get_device("led3");
    Leds[4] = wb_robot_get_device("led4");
    Leds[5] = wb_robot_get_device("led5");
    Leds[6] = wb_robot_get_device("led6");
    Leds[7] = wb_robot_get_device("led7");
    Leds[8] = wb_robot_get_device("led8");
    Leds[9] = wb_robot_get_device("led9");
    
    wb_led_set(Leds[0],-1);
    wb_led_set(Leds[1],-1);

  
   
   
   
  while (wb_robot_step(TIME_STEP) != -1) 
  {
    for(i=0;i<256;i++) texto[i]=0;


    /* for para controle de valores dos sensores no mundo */
    for(i=0;i<QtddSensoresProx;i++)
    {
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i])-60;
       sprintf(texto,"%s|%d: %5.2f  ",texto,i,LeituraSensorProx[i]);
       posicao = wb_supervisor_field_get_sf_vec3f(trans_field);

    }
    printf("%s\n",texto);
    printf("Posicao do robo: x= %f   y= %f z= %f\n", posicao[0], posicao[1], posicao[2]);
    // faz o led piscar
    //wb_led_set(Leds[0], wb_led_get(Leds[0])*-1);

    //if para capturar a colisão entre objetos no mundo. Se o sensor está alto, tem colisão no sensor.
    if(LeituraSensorProx[0]>353)
    {
      AceleradorDireito =  1; // ajuda na detecção de um objeto móvel, empurrando mais eles
      AceleradorEsquerdo = 1;
      
      x[aux] = posicao[0]; // preenche o vetor x
      z[aux] = posicao[2]; // preenche o vetor z 
      printf("OBJETO DETECADO EM x%d %f | z%d %f\n", aux,  x[aux], aux, z[aux]); //mostra o valor do eixo x e z no momento da colisão
      
      
      if(contador > 9 ) // para ajudar na detecção de colisões e para auxiliar na colisão de objetos móveis, utilizamos o contador para quando
                        // atingir um valor maior que 9, acionar o motor esquerdo.
      { 
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
        
        
        //se X maior que 0, significa que estamos em uma parte do eixo x de valores positivos onde aconteceu a colisão
        if(x[0] > 0)
        {
          if(x[2] > x[8])
          {
            eixoP = x[2] - x[8]; // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            
            if(eixoP > 0.0147)
            { // se distância maior que 0.0147 podemos concluir que o objeto mexeu 
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 0  <=================\n"); // indicação de objeto móvel pelo sensor 0
               cont_led += 1;
               cont_caixa_movel += 1;
            }
          }
          
          else
          {
            eixoP = x[8] - x[2]; // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            if(eixoP > 0.0147){ //se distância maior que 0.0147 podemos concluir que o objeto alterou sua posição no mundo, logo, é móvel.
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 0  <=================\n");  // indicação de objeto móvel pelo sensor 0
              cont_led += 1;
              cont_caixa_movel += 1;
            }
          }  
        }
        
        
        //se X menor que 0, significa que estamos em uma parte do eixo x de valores negativos onde aconteceu a colisão
        else if(x[0] < 0)
        { 
          if(x[2] > x[8])
          {
            eixoN = (x[8]) - (x[2]);  // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            if(eixoN < -0.0147)
            { // se distância maior que 0.0147 podemos concluir que o objeto alterou sua posição no mundo, logo, é móvel.
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 0  <=================\n");
               cont_led += 1;
               cont_caixa_movel += 1;
            }
          }
          
          
          else
          {
            eixoN = (x[2]) - (x[8]);  // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            if(eixoN < -0.0147)
            { // se distância maior que 0.0147 podemos concluir que o objeto alterou sua posição no mundo, logo, é móvel.
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 0  <=================\n");
               cont_led += 1;
               cont_caixa_movel += 1;
            }
          }  
        }
      }
      
      aux++;
      contador++;
      
     }
     
      //aciona sensores/motores quando identifica uma colisão nos sensores.
     else if(LeituraSensorProx[1]>530) 
     {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[2]>950) // acionameto dos motores quando o sensor atinge um valor alto
     {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[3]>950)  // acionameto dos motores quando o sensor atinge um valor alto
     {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[4]>950)  // acionameto dos motores quando o sensor atinge um valor alto
     {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[5]>950)  // acionameto dos motores quando o sensor atinge um valor alto
     {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[6]>530)  // acionameto dos motores quando o sensor atinge um valor alto
     {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
    else if(LeituraSensorProx[7]>353)  // acionameto dos motores quando o sensor atinge um valor alto
     {
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
     
      x[aux] = posicao[0]; // preenche o vetor x[]
      z[aux] = posicao[2]; // preenche o vetor z[]
      printf("OBJETO DETECTADO EM x%d %f | z%d %f\n", aux,  x[aux], aux, z[aux]); //mostra o valor do eixo x e z no momento da colisão
      
     if(contador > 9)
     { // para ajudar na detecção de colisões e para auxiliar na colisão de objetos móveis, utilizamos o contador para quando
                       // atingir um valor maior que 9, acionar o motor esquerdo.
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;

        
         //se X maior que 0, significa que estamos em uma parte do eixo x de valores positivos onde aconteceu a colisão
        if(x[0] > 0){
          if(x[2] > x[8])
          {
            eixoP = x[2] - x[8]; // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            
            if(eixoP > 0.0147)
            { // se distância maior que 0.0147 podemos concluir que o objeto alterou sua posição no mundo, logo, é móvel.
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 7  <=================\n");
               cont_led += 1;
               cont_caixa_movel += 1;
            }
          }
          
          else
          {
            eixoP = x[8] - x[2];  // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            if(eixoP > 0.0147)
            {  // se distância maior que 0.0147 podemos concluir que o objeto alterou sua posição no mundo, logo, é móvel.
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 7  <=================\n");
               cont_led += 1;
               cont_caixa_movel += 1;
            }
          }  
        }
        
         //se X menor que 0, significa que estamos em uma parte do eixo x de valores negativos onde aconteceu a colisão
        else if(x[0] < 0)
        { 
          if(x[2] > x[8])
          {
            eixoN = (x[8]) - (x[2]); // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            
            if(eixoN < -0.0147)
            { // se distância maior que 0.0147 podemos concluir que o objeto alterou sua posição no mundo, logo, é móvel.
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 7  <=================\n");
               cont_led += 1;
               cont_caixa_movel += 1;
            } 
          }
          
          else
          {
            eixoN = (x[2]) - (x[8]); // calcula a distância entre x[2] e x[8] ( sempre com valores positivos )
            if(eixoN < -0.0147)
            { // se distância maior que 0.0147 podemos concluir que o objeto alterou sua posição no mundo, logo, é móvel.
              printf("=================> CAIXA MÓVEL DETECTADA PELO SENSOR 7  <=================\n");
              cont_led += 1;
              cont_caixa_movel += 1;
            }
          }
        }
      }
      
      aux++;
      contador++;
     }
      
     else {  // faz o dev_robot andar em linha reta quando nenhum motor sensor é acionado ( colisões )
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
      aux=0;
      contador=0;
     }
      
 
    
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck"); //captura o supervisor
    wb_motor_set_velocity(MotorEsquerdo,6*AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6*AceleradorDireito);
    
   

   
   if(cont_led >= 2)
   {
       wb_led_set(Leds[0], 0);
       wb_led_set(Leds[1], 0);
       wb_led_set(Leds[2], 0);
       wb_led_set(Leds[3], 0);
       wb_led_set(Leds[4], 0);
       wb_led_set(Leds[5], 0);
       wb_led_set(Leds[6], 0);
       wb_led_set(Leds[7], 0);

     //acende todos os leds 3 vezes  
     for(int cont = 0; cont <= 3; cont++)
     {
        
       wb_led_set(Leds[0], 1); // = 1 ligado
       wb_led_set(Leds[1], 1); 
       wb_led_set(Leds[2], 1);
       wb_led_set(Leds[3], 1);
       wb_led_set(Leds[4], 1);
       wb_led_set(Leds[5], 1);
       wb_led_set(Leds[6], 1);
       wb_led_set(Leds[7], 1);
       wb_led_set(Leds[0], 0); // = 0 desligado
       wb_led_set(Leds[1], 0);
       wb_led_set(Leds[2], 0);
       wb_led_set(Leds[3], 0);
       wb_led_set(Leds[4], 0);
       wb_led_set(Leds[5], 0);
       wb_led_set(Leds[6], 0);
       wb_led_set(Leds[7], 0);
     }
   
   
     if(cont_acende_led == 0)
          wb_led_set(Leds[0], 1);
    
     if(cont_acende_led == 1)
     {
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
     }
     if(cont_acende_led == 2)
     {
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
     }
     if(cont_acende_led == 3)
     {
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
     }
     if(cont_acende_led == 4)
     {
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
     }
     if(cont_acende_led == 5)
     {
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
          wb_led_set(Leds[5], 1);
     }
     if(cont_acende_led == 6)
     {
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
          wb_led_set(Leds[5], 1);
          wb_led_set(Leds[6], 1);
     }
     cont_acende_led++;
     cont_led = 0;
     
   }
    
  };

  wb_robot_cleanup();

  return 0;
}


