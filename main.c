/*                             UTFORCE  E-RACING
 *               Thiago Fellipe Ortiz de Camargo
 *                          10/2018 
 *                    PIC18F26K83 - MCP2561
 *                De acordo com normas SAE 2018
 * 
 *      Programa para leitura da posição do pedal do acelerador composto por 2 
 *  sensores TPS comuns. De acordo com a norma, deve-se realizar 
 *  a verificação dos valores de leitura para o pedal do acelerador. Os potêncio-
 *  metros devem possuir função de transferencia diferente(OFFSET, gradiente).Por 
 *  este motivo utiliza-se Vref analógica.
 * 
 *      A comparação entre as duas curvas dos sensores foi efetuada tendo como referencia
 *  a diferença entre os dois coeficientes angulares das retas parametrizadas de 
 *  acordo com as posições iniciais e finais dos sensores. Assumindo essa diferença como 
 *  referencia, calcula-se a todo momento os coeficientes angulares respectivos as posições
 *  instantaneas dos sensores. Como são retas os coeficientes angulares se mantem os mesmos,
 *  caso aconteça interferencia na leitura, por qualquer motivo alheio a essa explicação,
 *  a derivada da equação dessas retas serão alteradas, distoando do valor de referencia.
 *
 *      Para proteção de curto circuito, onde 0,5V<Vin<4,5V, e por isso estes 
 *  valores de leitura analógica não sao aceitos.
 *      Leitura da posição do freio é realizada via encoder magnético modelo AS5040.
 *  De acordo com a norma, deve-se haver regiões em que o motor deve ligar ou 
 *  desligar, de acordo com a posição do pedal de freio. São elas:
 * 
 *  Estado 1 - Para valores menores que 10% e maiores que 90% do sinal APPS, deve-se cortar
 *  a energia para o motor. Esses valores de 10% e 90% podem vairiar de acordo com o projeto
 *  do curso do pedal box.
 * 
 *  Estado 2 - Quando o mecanismo do freio está acionado, com qualquer intensidade,
 *  e a intensidade do sinal do APPS estiver entre 25%-100%, deve-se cortar a energia do motor.
 *  O estado 2 deve ser mantido até que o sinal do freio seja 5% do seu total. Esta condição atentida
 *  o motor pode ser alimentado.
 *
 *      Comunicação com o inversor de frequencia(WEG CVW300) é por rede CAN(125kbps)
 *  por ordenação little endian.
 * 
 *  Node ID: 1
 *  Pacote de dados formado da seguinte maneira:
 *      Byte 0: LSB - Referencia de velocidade
 *      Byte 1: MSB - Referencia de velocidade
 *      Byte 2: LSB - Leitura Freio
 *      Byte 3: MSB - Leitura Freio
 * 
 *  Só se utiliza o buffer 0(maior prioridade)
 */

#include "mcc_generated_files/pin_manager.h"


#include "mcc_generated_files/mcc.h"


//Ready-To-Drive-Sound declarações
int iControlRTDSTimer, iContEstRTDS = 0;
uint16_t uiRTRTX = 1111, uiRTRRX = 0; // Valor arbitrário
uCAN_MSG uCAN_rxRTRB0;
uCAN_MSG uCAN_txRTRB0;
uCAN_MSG funcionaaimano;

//Coleta de dados para parametrização das retas
int iControleCaluloRetas = 0;
int icontroleDadosExternos = 0;
float fVglExtremo, fDplExtremo;
float uiVglInicio = 1000, uiVglFinal = 2800, uiDplInicio = 1000, uiDplFinal = 3000;

//Lógica do PedalBox declarações
int iContEstAcel = 0, iControleFreio = 0, iControleTimer1Acel = 0, iControleTimer2Acel = 0, iControleAcel = 0, iContEstAPPSImp = 0;
float fLimInfAcel, fLimSupAcel, fAcelerador2, fVerificacaoRetas, fFreioEstado2;
float fLimInfFreio, fLimSupFreio, fAcelerador1, uiVerificacaoFreio = 4097, fFreio;
unsigned int uiAcelerador1Can, a;
float uiLeitura1 = 1500, uiLeitura2 = 40000;
int iControleHabilita = 0;
int controle1 = 0;
int controle2 = 0;

//Aplcations
float fDiferencaAngular, fConstRef;
uCAN_MSG uCAN_ucMensagemB0;

// Memory
uint8_t iMemoryControl, uiVglInicioLB, uiVglInicioHB, uiVglFinalLB, uiVglFinalHB;
uint8_t uiDplInicioLB, uiDplInicioHB, uiDplFinalLB, uiDplFinalHB;
int iControleEntradaMemoria = 0;

/*void naosei() {

    funcionaaimano.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
    funcionaaimano.frame.id = 0x11; //Define o ID de envio (Abritrário)
    funcionaaimano.frame.dlc = 0x2; //Define o tamanho da mensagem em bytes
    funcionaaimano.frame.data0 = uiRTRTX & (0x00FF); //LITTLE ENDIAN
    funcionaaimano.frame.data1 = (uiRTRTX >> 8) & (0xFF); //LITTLE ENDIAN
    __delay_ms(50);
    if(CAN_messagesInBuffer()){
        CAN_transmit(&funcionaaimano);
        __delay_ms(50);
        LED0_Toggle();
    }
    if(CAN_isBusOff()){
        LED1_Toggle();
    }
    if(CAN_isTXErrorPassive()) {
        LED2_Toggle();
    }
    LED3_Toggle();

}*/

void StarButton_RTDS(void) {
    
    //LED1_Toggle();
    //static int controle = 0;

    fFreio = (float) ADCC_GetSingleConversion(FREIO);

    // Condição prevista em norma para colocar o carro em Ready-To-Drive-Mode
    if (fFreio >= 450) { // Próximo ao limite do curso do freio (818.4)

        LED1_Toggle();

        /*funcionaaimano.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
        funcionaaimano.frame.id = 0x11; //Define o ID de envio (Abritrário)
        funcionaaimano.frame.dlc = 0x2; //Define o tamanho da mensagem em bytes
        funcionaaimano.frame.data0 = uiRTRTX & (0x00FF); //LITTLE ENDIAN
        funcionaaimano.frame.data1 = (uiRTRTX >> 8) & (0xFF); //LITTLE ENDIAN

        CAN_transmit(&funcionaaimano);
        __delay_ms(50);*/
        controle1++;
    }

    

    EXT_INT1_InterruptFlagClear();  //Esta flag não é limpa, pois esse evento deve acontecer apenas uma vez, para que ocorra novamente é necessário reboot do sistema
}

void ParametrizacaoRetas() {

    // Parametrização das curvas acelerador
    float aux1, aux2;
    //Equações da reta
    aux1 = 100 * (uiLeitura1 - uiVglInicio);
    fAcelerador1 = aux1 / (uiVglFinal - uiVglInicio);
    if (fAcelerador1 <= 1) {
        fAcelerador1 = 0;
        fAcelerador2 = 4444;
    }
    if (fAcelerador1 > 105)fAcelerador2 = 4444;


    aux2 = 100 * (uiLeitura2 - uiDplInicio);
    fAcelerador2 = aux2 / (uiDplFinal - uiDplInicio);
    if (fAcelerador2 <= 1) {
        fAcelerador2 = 0;
        fAcelerador2 = 4444;
    }
    if (fAcelerador2 > 105)fAcelerador2 = 4444;

    if (ADCC_GetSingleConversion(TPS1) > uiVglFinal) {
        fAcelerador2 = 4444;
    }

    //Define-se o Acelerador2 (DPL) como refeência para o inversor
    uiAcelerador1Can = (unsigned int) fAcelerador2;

}

void CalculoExtremos() {

    //Para que não haja indeterminação utilizamos a imagem (referência de torque)
    //como 150 nos calculos

    //Utilizando a Equação da reta achamos o ponto do dominio para que a imagem seja 150

    fVglExtremo = (150 * (uiVglFinal - uiVglInicio) + 100 * uiVglInicio)*0.01;

    fDplExtremo = (150 * (uiDplFinal - uiDplInicio) + 100 * uiDplInicio)*0.01;

}

float CalculoConstante() {
    // Calculo da diferença entre os coenficientes angulares, padrões, das retas dos sensores TPS's

    float fConstante;

    fConstante = ((100 / (uiVglFinal - uiVglInicio)) - (100 / (uiDplFinal - uiDplInicio)));


    return fConstante;
}

float CalculoConstanteReferenciaLeitura(float fDiferencaAngular) {

    //Esta função calcula a constante que deve ser comparada com a ConstanteReferenciaLeitura e validada

    float fConstRef, fAux1, fAux2, fAux3, fAux4, fAux5, fAux6, fAux7;

    fAux1 = (150 - fAcelerador1) / (fVglExtremo - uiLeitura1);
    fAux2 = (150 - fAcelerador2) / (fDplExtremo - uiLeitura2);
    fConstRef = fAux1 - fAux2;


    /*fAux1 = uiLeitura1*(fDiferencaAngular*fDplExtremo-150);
    fAux2 = uiLeitura2*(fDiferencaAngular*fVglExtremo+150);
    fAux3 = (-1)*fAcelerador2*fVglExtremo;
    fAux4 = fAcelerador2*uiLeitura1;
    fAux5 = fAcelerador1*fDplExtremo;
    fAux6 = (-1)*fAcelerador1*uiLeitura2;
    fAux7 = (-1)*fDiferencaAngular*uiLeitura1*uiLeitura2;
    
    fConstRef = fAux1 + fAux2 + fAux3 + fAux4 + fAux5 + fAux6 + fAux7;*/


    return fConstRef;
}

float CalculoConstanteReferenciaEstatica(float fDiferencaAngular) {

    //Esta função calcula a constante que é tomada como referência na comparação à ConstanteReferenciaLeitura

    float ConstanteEstatica, fAux1, fAux2, fAux3;

    fAux1 = (150) / (fVglExtremo - uiVglInicio);
    fAux2 = 150 / (fDplExtremo - uiDplInicio);

    ConstanteEstatica = fAux1 - fAux2;



    /*fAux1 = fDiferencaAngular*fVglExtremo*fDplExtremo;
    fAux2 = 150*fDplExtremo;
    fAux3 = -150*fVglExtremo;
    
    ConstanteEstatica = fAux1 + fAux2+ fAux2;*/




    return ConstanteEstatica;

}
float Filter (){
    //Multisampling
}
void Aplications() {

    while (1)
    {


        iMemoryControl = DATAEE_ReadByte(0x00);

        if(iMemoryControl == 1 && iControleEntradaMemoria == 0) 
        {
            iControleCaluloRetas = 1;
            uiVglInicio = DATAEE_ReadByte(0x01) | (DATAEE_ReadByte(0x02) << 8);
            uiVglFinal = DATAEE_ReadByte(0x03) | (DATAEE_ReadByte(0x04) << 8);
            uiDplInicio = DATAEE_ReadByte(0x05) | (DATAEE_ReadByte(0x06) << 8);
            uiDplFinal = DATAEE_ReadByte(0x07) | (DATAEE_ReadByte(0x08) << 8);
            iMemoryControl = 0;
            iControleEntradaMemoria = 1;
        }


        //iControleHabilita = 1;

        if (iControleCaluloRetas == 1 && iControleHabilita == 1) {
            //LED3_Toggle();
            
            int n_de_amostras_adequadas;
            
            uiLeitura1 = (float) ADCC_GetSingleConversion(TPS1); //AN0 VGL
            for (int i = 0; i < n_de_amostras_adequadas; i++){
                uiLeitura1 += (float) ADCC_GetSingleConversion(TPS1);
                }
                uiLeitura1 = uiLeitura1 / n_de_amostras_adequadas;
            uiLeitura2 = (float) ADCC_GetSingleConversion(TPS2); //AN1 DPL
            for (int i = 0; i < n_de_amostras_adequadas; i++){
                uiLeitura2 += (float) ADCC_GetSingleConversion(TPS2);
                }
                uiLeitura2 = uiLeitura2 / n_de_amostras_adequadas;
            fFreio = (float) ADCC_GetSingleConversion(FREIO); //AN4
            for (int i = 0; i < n_de_amostras_adequadas; i++){
                fFreio += (float) ADCC_GetSingleConversion(FREIO);
                }
                fFreio = fFreio / n_de_amostras_adequadas;

            ParametrizacaoRetas();
            CalculoExtremos();
            fDiferencaAngular = CalculoConstante();
            fConstRef = CalculoConstanteReferenciaLeitura(fDiferencaAngular);
            fVerificacaoRetas = CalculoConstanteReferenciaEstatica(fDiferencaAngular);

            fLimInfAcel = (fVerificacaoRetas - (fVerificacaoRetas * 0.1));
            fLimSupAcel = (fVerificacaoRetas + (fVerificacaoRetas * 0.1));

            // VERIFICAÇÃO SENSOR Freio - Varia de acordo com o projeto do pedalbox//

            fLimInfFreio = 4; //(DEFINIR VALORES)
            fLimSupFreio = 4096; //(DEFINIR VALORES)

            fFreioEstado2 = ((0.05*(fLimSupFreio-fLimInfFreio)))+fLimInfFreio; // 5% do cruso do pedal do freio ( ~ 256)


            // Estruturas de verificação //


            if (fConstRef >= fLimInfAcel && fConstRef <= fLimSupAcel) {
                iControleTimer1Acel = 0;
                iContEstAPPSImp = 0;
                TMR1_StopTimer();
                
                //LED1_Toggle();
                //__delay_ms(50);

                // Lógica Freio //

                //Estado 1//


                if (fFreio >= fLimInfFreio && fFreio <= fLimSupFreio) {

                    

                    // +20 margem para teste / 25 = 25% do curso, acelerador, dentro dos limites superor e inferior
                    if ((fFreio > (fLimInfFreio + 30)) && (fAcelerador1 >= 25) && iControleFreio == 0) {

                        iControleFreio = 1;
                        

                        // Parametros para corte de energia do motor//

                        uiAcelerador1Can = 4444;



                    }
                    else if (iControleFreio == 1) {


                        if (fAcelerador1 <= 5) {

                            iControleFreio = 0;
                            uiAcelerador1Can = (unsigned int) fAcelerador2;
                            
                            
                            
                            uCAN_txRTRB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
                            uCAN_txRTRB0.frame.id = 0x4; //Define o ID de envio (Abritrário)
                            uCAN_txRTRB0.frame.dlc = 0x1; //Define o tamanho da mensagem em bytes
                            uCAN_txRTRB0.frame.data0 = 0; //LITTLE ENDIAN
                            CAN_transmit(&uCAN_txRTRB0);
                            __delay_ms(50);
                            
                            uCAN_txRTRB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
                            uCAN_txRTRB0.frame.id = 0x1; //Define o ID de envio (Abritrário)
                            uCAN_txRTRB0.frame.dlc = 0x2; //Define o tamanho da mensagem em bytes
                            uCAN_txRTRB0.frame.data0 = uiRTRTX & (0x00FF); //LITTLE ENDIAN
                            uCAN_txRTRB0.frame.data1 = (uiRTRTX >> 8) & (0xFF); //LITTLE ENDIAN
                            CAN_transmit(&uCAN_txRTRB0);
                            
                            //LED0_Toggle();
                            
                        }
                        else {
                            uiAcelerador1Can = 4444;
                        }
                        
                    }

                    else {

                        iControleFreio = 0;

                        uiAcelerador1Can = (unsigned int) fAcelerador2;

                        uCAN_txRTRB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
                        uCAN_txRTRB0.frame.id = 0x4; //Define o ID de envio (Abritrário)
                        uCAN_txRTRB0.frame.dlc = 0x1; //Define o tamanho da mensagem em bytes
                        uCAN_txRTRB0.frame.data0 = 0; //LITTLE ENDIAN
                        CAN_transmit(&uCAN_txRTRB0);
                        
                        __delay_ms(50);
                        uCAN_txRTRB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
                        uCAN_txRTRB0.frame.id = 0x1; //Define o ID de envio (Abritrário)
                        uCAN_txRTRB0.frame.dlc = 0x2; //Define o tamanho da mensagem em bytes
                        uCAN_txRTRB0.frame.data0 = uiRTRTX & (0x00FF); //LITTLE ENDIAN
                        uCAN_txRTRB0.frame.data1 = (uiRTRTX >> 8) & (0xFF); //LITTLE ENDIAN
                        CAN_transmit(&uCAN_txRTRB0);
                        
                        
                        
                      
                        //LED1_Toggle();
                    }

                }
                else {

                    // pull down/up resistor sensores acelerador // 
                    // Parametros para corte de energia do motor//
                    uiAcelerador1Can = 4444;

                }

            }
            else {

                /*if(iControleTimer1Acel == 0){
                    LED2_Toggle();
                    TMR1_StartTimer();
                    iControleTimer1Acel = 1;
                }
                
                
                if(iControleTimer2Acel == 1){

                    // Parametros para corte de energia do motor//

                    uiAcelerador1Can = 4444;
                    TMR1_StopTimer();
                    iControleTimer1Acel = 0;

                }*/
                uiAcelerador1Can = 4444;


            }
            
            uCAN_ucMensagemB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
            uCAN_ucMensagemB0.frame.id = 0x2; //Define o ID de envio (Abritrário)
            uCAN_ucMensagemB0.frame.dlc = 0x2; //Define o tamanho da mensagem em bytes
            uCAN_ucMensagemB0.frame.data0 = uiAcelerador1Can & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data1 = (uiAcelerador1Can >> 8) & (0xFF); //LITTLE ENDIAN
            CAN_transmit(&uCAN_ucMensagemB0); //Envia a mensagem
            __delay_ms(50);
            
            int x = fFreio;
            int y = uiLeitura1;
            int z = uiLeitura2;
            
            uCAN_ucMensagemB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
            uCAN_ucMensagemB0.frame.id = 0x11; //Define o ID de envio (Abritrário)
            uCAN_ucMensagemB0.frame.dlc = 0x6; //Define o tamanho da mensagem em bytes
            uCAN_ucMensagemB0.frame.data0 = x & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data1 = (x >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data2 = y & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data3 = (y >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data4 = z & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data5 = (z >> 8) & (0xFF); //LITTLE ENDIAN
            CAN_transmit(&uCAN_ucMensagemB0); //Envia a mensagem
            __delay_ms(50);
            
            int x1 = uiVglInicio;
            int x2 = uiDplInicio;
            int x3 = fConstRef;
            int x4 = fAcelerador1;
            uCAN_ucMensagemB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
            uCAN_ucMensagemB0.frame.id = 0x12; //Define o ID de envio (Abritrário)
            uCAN_ucMensagemB0.frame.dlc = 0x8; //Define o tamanho da mensagem em bytes
            uCAN_ucMensagemB0.frame.data0 = x1 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data1 = (x1 >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data2 = x2 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data3 = (x2 >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data4 = x3 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data5 = (x3 >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data6 = x4 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data7 = (x4 >> 8) & (0xFF); //LITTLE ENDIAN
            CAN_transmit(&uCAN_ucMensagemB0); //Envia a mensagem
            __delay_ms(50);
            
            int y1 = uiVglInicio;
            int y2 = uiDplInicio;
            int y3 = fConstRef;
            int y4 = fAcelerador1;
            
            uCAN_ucMensagemB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
            uCAN_ucMensagemB0.frame.id = 0x13; //Define o ID de envio (Abritrário)
            uCAN_ucMensagemB0.frame.dlc = 0x8; //Define o tamanho da mensagem em bytes
            uCAN_ucMensagemB0.frame.data0 = x1 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data1 = (x1 >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data2 = x2 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data3 = (x2 >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data4 = x3 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data5 = (x3 >> 8) & (0xFF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data6 = x4 & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data7 = (x4 >> 8) & (0xFF); //LITTLE ENDIAN
            CAN_transmit(&uCAN_ucMensagemB0); //Envia a mensagem
            __delay_ms(50);
            //LED2_Toggle();

        }
        else {

            uiAcelerador1Can = 4444;


            uCAN_ucMensagemB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
            uCAN_ucMensagemB0.frame.id = 0x4; //Define o ID de envio (Abritrário)
            uCAN_ucMensagemB0.frame.dlc = 0x2; //Define o tamanho da mensagem em bytes
            uCAN_ucMensagemB0.frame.data0 = uiAcelerador1Can & (0x00FF); //LITTLE ENDIAN
            uCAN_ucMensagemB0.frame.data1 = (uiAcelerador1Can >> 8) & (0xFF); //LITTLE ENDIAN
            CAN_transmit(&uCAN_ucMensagemB0); //Envia a mensagem
            __delay_ms(50);
            //LED3_Toggle();

        }


    }

}

void SetPedalSettings(void) {

    __delay_ms(200); // Delay para auxiliar na redução de captação de ruido gerado pelo botão

    switch (icontroleDadosExternos) {
        case 0:

            icontroleDadosExternos = 1;

            //coleta dados inicio do curso do 
            uiVglInicio = (float) ADCC_GetSingleConversion(TPS1); //Leitura analogica dos TPSs 

            uiDplInicio = (float) ADCC_GetSingleConversion(TPS2);

            LED1_SetHigh();
            __delay_ms(300); //Alerta de coleta
            LED1_SetLow();
            break;

        case 1:

            icontroleDadosExternos = 2;

            //coleta dados fim do curso do pedal              

            uiVglFinal = (float) ADCC_GetSingleConversion(TPS1); //Leitura analogica dos TPSs

            uiDplFinal = (float) ADCC_GetSingleConversion(TPS2);

            LED2_SetHigh();
            __delay_ms(300); //Alerta de coleta
            LED2_SetLow();
            break;


        case 2:

            icontroleDadosExternos++;
            iControleCaluloRetas = 1;



            LED1_SetHigh();
            LED2_SetHigh();
            LED3_SetHigh();
            __delay_ms(300); //Alerta de coleta concluida
            LED1_SetLow();
            LED2_SetLow();
            LED3_SetLow();
            //EXT_INT0_InterruptFlagClear();

            // Armazenamento de dados de posição TPSs na memoria EEPROM

            //VGL
            uiVglInicioLB = (uint16_t) uiVglInicio & (0x00FF); //LITTLE ENDIAN
            uiVglInicioHB = ((uint16_t) uiVglInicio >> 8) & (0xFF); //LITTLE ENDIAN

            DATAEE_WriteByte(0x01, uiVglInicioLB);
            DATAEE_WriteByte(0x02, uiVglInicioHB);

            uiVglFinalLB = (uint16_t) uiVglFinal & (0x00FF); //LITTLE ENDIAN
            uiVglFinalHB = ((uint16_t) uiVglFinal >> 8) & (0xFF); //LITTLE ENDIAN

            DATAEE_WriteByte(0x03, uiVglFinalLB);
            DATAEE_WriteByte(0x04, uiVglFinalHB);


            //DPL

            uiDplInicioLB = (uint16_t) uiDplInicio & (0x00FF); //LITTLE ENDIAN
            uiDplInicioHB = ((uint16_t) uiDplInicio >> 8) & (0xFF); //LITTLE ENDIAN

            DATAEE_WriteByte(0x05, uiDplInicioLB);
            DATAEE_WriteByte(0x06, uiDplInicioHB);

            uiDplFinalLB = (uint16_t) uiDplFinal & (0x00FF); //LITTLE ENDIAN
            uiDplFinalHB = ((uint16_t) uiDplFinal >> 8) & (0xFF); //LITTLE ENDIAN

            DATAEE_WriteByte(0x07, uiDplFinalLB);
            DATAEE_WriteByte(0x08, uiDplFinalHB);

            //Variavel de controle

            iMemoryControl = 1;
            DATAEE_WriteByte(0x00, iMemoryControl);


            //Aplications();

            break;

        case 3:
            //Reboot

            icontroleDadosExternos = 0;
            uiVglInicio = 0;
            uiVglFinal = 0;
            uiDplInicio = 0;
            uiDplFinal = 0;

            //Alerta de reboot
            LED1_SetHigh();
            __delay_ms(150);
            LED1_SetLow();
            __delay_ms(150);
            LED2_SetHigh();
            __delay_ms(150);
            LED2_SetLow();
             __delay_ms(150);
            LED3_SetHigh();
            __delay_ms(150);                                          
            LED3_SetLow();


            break;
    }

    EXT_INT0_InterruptFlagClear();

}

void TimerRTDStart(void) {

    //LED1_Toggle();
    TMR0_StartTimer();
    EXT_INT2_InterruptFlagClear();
    iControleHabilita = 1;

}

void RTDSTime(void) {

    LED4_SetHigh();

    iContEstRTDS++; //Variável de controle de tempo

    if (iContEstRTDS == 450) { //1.8 + erro seg
        Rele_RTDS_SetLow(); //Desliga o RTDS
        LED4_SetLow();
        TMR0_StopTimer(); //Desliga o Timer 0
    }

    PIR3bits.TMR0IF = 0;

}

void APPSImplausibility(void) {
    LED1_Toggle();
    iContEstAPPSImp++;

    if (iContEstAPPSImp == 10) { //100ms
        iControleTimer2Acel = 1;
        iContEstAPPSImp = 0;
        TMR1_StopTimer();
    }

    PIR4bits.TMR1IF = 0;
}

void main(void) {
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INT1_SetInterruptHandler(StarButton_RTDS);
    INT0_SetInterruptHandler(SetPedalSettings);
    INT2_SetInterruptHandler(TimerRTDStart);
    EXT_INT0_risingEdgeSet();
    EXT_INT1_risingEdgeSet();
    EXT_INT2_risingEdgeSet();
    TMR0_Initialize();
    TMR0_SetInterruptHandler(RTDSTime);
    TMR0_StopTimer();
    TMR1_Initialize();
    TMR1_SetInterruptHandler(APPSImplausibility);
    TMR1_StopTimer();

    PIE5bits.RXB0IE = 1;
    PIR5bits.RXB0IF = 0;
    INTCON0bits.GIEL = 1;

    while (1) {
        while((iControleHabilita != 1) && (controle1 != 0)){
        uCAN_ucMensagemB0.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
        uCAN_ucMensagemB0.frame.id = 0x4; //Define o ID de envio (Abritrário)
        uCAN_ucMensagemB0.frame.dlc = 0x1; //Define o tamanho da mensagem em bytes
        uCAN_ucMensagemB0.frame.data0 = 0 & (0x00FF); //LITTLE ENDIAN
        CAN_transmit(&uCAN_ucMensagemB0); //Envia a mensagem
          
            //LED3_Toggle();
            
         LED2_Toggle();
        __delay_ms(50);
        funcionaaimano.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //Define o tipo do ID (2.0B)        
        funcionaaimano.frame.id = 0x1; //Define o ID de envio (Abritrário)
        funcionaaimano.frame.dlc = 0x2; //Define o tamanho da mensagem em bytes
        funcionaaimano.frame.data0 = uiRTRTX & (0x00FF); //LITTLE ENDIAN
        funcionaaimano.frame.data1 = (uiRTRTX >> 8) & (0xFF); //LITTLE ENDIAN
        CAN_transmit(&funcionaaimano);
        controle2 = 1;
        Button_LED_SetHigh();
        
       
        
      }
        LED3_SetHigh();
        __delay_ms(30);
        LED3_SetLow();
        __delay_ms(30);
        LED3_SetHigh();
        __delay_ms(30);
        LED3_SetLow();
        __delay_ms(300);
        if (controle2 == 1){
          Aplications();  
        }
       // Aplications();
    }
        
    
}

