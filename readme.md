# Projeto

## Desafio
  Temos o objetivo de construir um algoritmo que mova um robô em um mapa de um ponto inicial ao seu objetivo no menor tempo.
  Disponibilizamos para isso de um robô com rodas e motor, com um sonar como um dispositivo de navegação.
  O espaço em que o robô está situado é desconhecido a ele.

## Implementação
  Um solução possível possível e ótima para problemas de distancias é o algoritmo A*, no qual é usado na navegação em nós, existem os desafios para a implementação do algoritmo neste problema: temos um espaço continuo. Para isto teremos que discretizar do nosso espaço, transformando-o em pedaços (tiles), idealmente um tile terá o tamanho do robô, pois este não pode passar por lugares menores que seu tamanho, porem pode ser expandida para ganho de perfomance com pouco custo em velocidade de solução.

  Outro problema de se implementar A* é que não sabemos (ou melhor, o robô) como é o ambiente aonde ele esta situado em relação a obstáculos, logo não é possível obter uma solução A* logo de inicio, teremos que fazer reconhecimento da area e para isso utilizar um array 2D da area que explorarmos.

  O Array 2D é inicializado com todos valores "0", automaticamente o robo explora a area ao seu arredor e mapeia o espaço na array.

  Implementaremos entao um algoritmo A* calculando a melhor rota do objetivo atual a o principal contando com os obstaculos existentes. A cada curva o algoritmo refaz a rota porque é de alta probabilidade que se encontre mais obstaculos novos, nao considerados, o robo recalculará tambem se este tiver movido alem do seu alcance do sensor.
