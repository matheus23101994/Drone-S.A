# Controle-Digital
Implementação de um controlador PID

Projeto de pesquisa a ser apresentado e submetido à avaliação para elaboração de Trabalho acadêmico do Curso de Engenharia de Computação e Engenharia de Controle e Automação do Centro de Ciências Tecnológicas da Universidade de Fortaleza. 

Orientador:	Afonso Henriques Fontes

## INTRODUÇÃO
Com o intermédio de todos os atuadores, controladores e sensores, será proposto um projeto  de um seguidor solar de 2 eixos, como o próprio nome diz simplesmente seguir o sol. O LDR (Sensor de luminosidade) será responsável por captar a quantidade de luz emitida pelo sol ou quaisquer fontes luminosas, os dados serão enviados ao controlador que por meio de lógica de programação, fara os servo motores atuarem de modo que a pirâmide, como no caso, fique sempre na melhor posição para a captação dos raios solares ou luminosos quaisquer.

O principal diferencial desse sistema comparado ao sistema convencional é o ganho de qualidade e produtividade na geração de energia solar, pois os painéis estarão sempre posicionados verticalmente para o sol. O gráfico a seguir mostra a eficiência na produção de energia dos dois sistemas:

## METODOLOGIA
Em nosso projeto os únicos atuadores existentes são os Servo motores. O modelo dos Servo motores usados foram o Tower Pro 9g. Os sensores utilizados foram quatro LDRs (Light Dependent Resistor). O controlador usado no projeto foi o Arduino UNO, onde o mesmo possui as seguintes especificações técnicas: 
•	Ultiliza microcontrolador Atmel
•	Possui tensão de funcionamentos de 5V
•	Tensão de entrada recomendável entre 7 - 12V
•	Possui 14 entradas/saídas digitais, entre as quais 	6 são PWM
•	Possui 6 pinos de entrada analógica
•	A corrente DC a ser utilizada nas entradas digitais não deve ser superior a 40mA
•	Possui 32Kb de memória flash
•	Possui 2Kb de SRAM
•	Possui 1Kb de EEPROM
•	E por fim ultiliza clock de 16 MHz

Foram utilizados os seguintes componentes no projeto:

Componente                            |Custo aproximado de unidade      |Custo aproximado
--------------------------------------|---------------------------------|---------------------------------
2 Potenciômetros	                    |R$ 2,00	                        |R$ 4,00
Cabos do tipo Jump	                  |-	                              |-
4 LDRs	                              |R$ 1,00	                        |R$ 4,00
4 Resistores de 10kΩ	                |R$ 0,10	                        |R$ 0,40
1 Suporte para servomotores Pan Tilt	|R$ 16,00	                        |R$ 16,00
1 Botoeira do tipo alavanca	          |R$ 3,00	                        |R$ 3,00
1 Protoboard	                        |R$ 20,00	                        |R$ 20,00

Custo total: R$ 47,40

## FUNCIONAMENTO DE CADA ITEM
-	LDR – Elemento do tipo passivo onde possível característica de um resistor variável, onde sua resistência varia conforme a intensidade de luz incide. Em nosso projeto foi implantados 4 LDR’s onde os mesmo funcionam aos pares por meio de simples comparação lógica e assim coordenando os eixos horizontal e vertical para melhor posicionamento com relação a luminosidade.
-	Servomotor – Maquina eletromecânica, onde o mesmo realiza movimento proporcional a um comando dado. Baseado nos dados obtidos com os LDR’s, o controlador irá executar a logica, onde resultado irá direcionar o posicionamento horizontal e vertical dos servomotores.
-	Arduino Uno – Plataforma de prototipagem eletronicade hardware livre, onde o mesmo possui um controlador Atmel com suporte de entradas e saídas embutido e possui um linguagem de programação que é essencialmente o C/C++. No projeto o controlador é responsável por diversas atividades logicas entre elas, as principais são:
-- Fazer leitura de dados dos LDR’s
-- Execução de lógica com aplicação do controlador PID
-- Controlar o posicionamento dos servomotores
-	Potenciômetro – Componente eletrônico que possui resistência elétrica ajustável. Geralmente os mais utilizados em projetos são os que possuem derivação central, onde o mesmo é deslizante e manipulável. Os potenciômetros utilizados desempenham o papel de controle manual de posicionamento do conjunto, sendo que: o potenciômetro 01 realiza o controle horizontal e o potenciômetro 02 que realiza o controle vertical.
-	Cabos do tipo Jump – Responsável pela transmissão de dados e corrente elétrica no projeto.
-	Suporte para servomotores Pan Tilt: Plataforma móvel usada para diversas aplicações, entre elas: Instalação de pequenas câmeras, sensores diversos, aeromodelismo, etc. Em nosso projeto a Pan Tilt é equipada com os servomotores, o qual tem a função de movimentar o conjunto equipado com os LDR’s.
-	Botoeira do tipo alavanca: Botão com trava do tipo alavanca, utilizado para desempenhar o papel de seleção entre controle automático e manual.
-	Protoboard: Placa de ensaio com furos, no caso utilizada para comportar todo os circuito.
 
## EMBASAMENTO TEÓRICO
O fluxograma a seguir apresenta o funcionamento lógico do seguidor solar:

O projeto foi montado conforme o diagrama esquemático abaixo:

O diagrama de blocos do sistema consiste em duas partes, a analógica, que envolve o Arduino e o Setpoint definido dentro do programa, e a digital que é toda a parte mecânica do projeto.

Abaixo seguem graficos com regressão dos potenciômetros e sensores. O mapeamento dos sinais de entrada com conversão para entrada linear foi descartada porque consideramos o sinal de entrada como apenas a diferença da luminosidade entre dois LDRs de cada direção.

## CONCLUSÃO
Com a crescente demanda de energias renováveis mais eficientes no mercado, o sistema de seguidor solar será concebido como uma solução muito benéfica.

O protótipo de seguidor solar o qual desenvolvemos é caracterizado pela a simplicidade e pela versatilidade de sua implementação, podendo ser facilmente empregado em sistemas reais de projetos de energia fotovoltaica. Com tudo existem claras limitações para o emprego real, pois dependendo das proporções do projeto, pode tornar-se algo relativamente inviável devido ao alto de custo de seu emprego, isso por conta da produção em escala dos equipamentos necessários para a automatização.

Com a implementação do controlador PID foi visivelmente notório a redução de erros de posicionamento, comparados a sistemas que não utilizam essa técnica de controle em seu conjunto. Notou-se também suavidade em seu posicionamento reduzindo obviamente desgastes tanto do suporte, quanto dos servomotores.

No entanto os controladores integral e derivativo foram menos atuantes no processo, sendo observado uma maior atuação por parte do controlador proporcional. O controlador proporcional trouxe velocidade ao processo, o controlador integrativo fez com o que o sistema reduzisse a sua velocidade quando aproximava-se do setpoint, o controlador diferencial trouxe estabilidade ao sistema.
