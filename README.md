# Sistema autônomos - Projeto de pesquisa e desenvolvimento de um quadricóptero

O projeto tem como objetivo o desenvolvimento de um quadricóptero que, através da aplicação de controle, é capaz de se manter estável e manter sua posição no espaço corrigindo interferências de perturbações nele aplicadas durante o vôo.
Durante o desenvolvimento foram definidas algumas atividades, distribuídas nas seguintes etapas:
1.	Realizar a construção do programa e o distribui-lo em bibliotecas;
2.	Obter valores brutos de leituras do MPU6050;
3.	Implementar o filtro de Kalman para leitura do MPU6050;
4.	Realizar a construção de uma estrutura para testes utilizando apenas dois motores;
5.	Configuração de uma interface de comunicação Wifi, utilizando um terminal TCP/IP;
6.	Realizar com o ajuste dos ganhos Kp, Ki e Kd a estabilização da estrutura;
7.	Montagem da estrutura com quatro motores;
8.	Realizar com o ajuste dos ganhos Kp, Ki e Kd, dos eixos Pitch, Roll e Yaw, a estabilização da estrutura;

A organização do código em bibliotecas foi de fundamental importância devido a sua complexidade e extensão do programa com um todo, dessa forma os erros durante a execução eram facilmente identificados corrigidos.
O MPU6050 é o sensor, acelerômetro, giroscópio que dá informações necessárias para medir a posição angular dos eixos do seu módulo e então do drone. A obtenção dos valores brutos do MPU6050 tem sua importância unicamente em conferir o correto funcionamento do módulo.
A implementação do filtro de Kalmann torna-se necessário pois as leituras obtidas a partir do MPU6050 vem acompanhadas de bastante ruído, necessitando assim de um filtro para obtenção valores mais estáveis conforme o manipulamos. 
A estrutura montada inicialmente contempla apenas a utilização de dois motores, um rotacionando no sentindo horário e outro no sentido anti-horário, com o objetivo de testar a eficiência dos motores e a realização do ajuste de ganhos em tempo real. Após o ajuste dos ganhos foi possível chegar a uma estabilidade razoavelmente considerável, onde as perturbações causadas eram corrigidas em um curto período de tempo pelo controlador.
Inicialmente a interface de comunicação utilizada foi construída utilizando http, porem com o decorrer da construção do programa, foram notórios alguns erros causados por este protocolo de comunicação que causava o funcionamento incorreto do restante do programa. A correção foi feita a partir da utilização do protocolo de comunicação tcp/ip, onde o mesmo oferece uma maior confiabilidade dos dados recebidos, como também a fluidez do funcionamento do programa, como também foi dispensado a utilização de um software exclusivo para o funcionamento, sendo substituído apenas por um terminal tcp/ip mobile baixado na play store.
Inicialmente os valores dos ganhos eram ajustados na programação a cada teste, mas com a implementação da comunicação tcp/ip ficou possível fazer os ajustes dos ganhos em tempo real via wi-fi.
O ajuste de ganhos pode ser um grande impasse ao funcionamento do quadricóptero, porem com algumas pesquisas realizadas chegamos a algumas dicas, que talvez, possa ser bastante útil:
1.	Ganho proporcional
•	Aumentar o Kp em passos de 0,2
•	Aumentar o Kp até o drone oscilar
•	Por fim, baixar o valor atual em 50%
2.	Ganho integral
•	Aumentar o Ki em passos de 0,01
•	Aumentar o Ki até o drone oscilar
•	Por fim baixar o valor atual em 50%
3.	Ganho derivativo
•	Aumentar o Kd até o drone oscilar
•	Diminuir o valor de Kd até o drone parar de oscilar
•	Por fim, baixar o valor atual em 25%





