# wally_control_odom

Este pacote contém os scripts de controle e odometria do robô Wally.

## Informações sobre os scripts

* **listenner_ticks:** Recebe a string que contém os valores de pulsos dos encoders do robô e os publica de forma individual a cada encoder (A, B, C e D) e também a média dos encoders da esquerda e da direita.

* **due_fake:** Recebe comandos do joystick e imita a publicação de dados dos encoders, com comandos do robô ir para frente, para trás, para esquerda e para a direita. Este script pode ser usado para controlar o Wally virtual por meio do joystick, sem a necessidade de acionamentos dos motores do Wally real. Os comandos de controle são: O analógico da esquerda controla as direções para frente, para trás, para a esquerda e para a direita, compativeis para o lado na qual o analógico for direcionado, com velocidade determinada pelo deslocamento do analógico. E o botão X zera as variáveis de contagem de pulsos. Ele foi criado para testes de odometria sem a necessidade do uso dos motores do Wally, economizando tempo, energia e o hardware na hora dos testes.

* **odometry:** Recebe os valores de pulso de cada encoder para determinar a posição e velocidade do robô.

* **odom:** Código de odometria escrito em python.

* **control_odom:** Realiza a odometria, faz o controle dos motores de forma individual e publica dados para análise gráfica da velocidade de cada motor. O controle é feito usando PID, em que é calculado o pwm necessário para que o robô atinja a velocidade de referência determinada, na qual o script também pode receber os parametros Kp, Ki, e Kd se for desejado, para sintonia do controle.

* **kp_set:** Responsável por receber do usuário e enviar para o nó de controle o parâmetro proporcional do controle PID.

* **ki_set:** Responsável por receber do usuário e enviar para o nó de controle o parâmetro integrativo do controle PID.

* **kd_set:** Responsável por receber do usuário e enviar para o nó de controle o parâmetro diferencial do controle PID.

### Informações sobre os scripts do arduino

* **wally_main_joy_rate:** Pode receber tanto comando do joystick como também comandos de velocidade para cada motor individualmente. E com este código, o arduino fazerá a leitura dos encoders de cada roda e publicará em uma string as quatro leituras com frequência de 20Hz. Os comandos do joy são: Botões A, B, X e START, param os motores, Y zera os contadores de pulsos e o botão BACK ativa e desativa comandos de movimento do joy (deve-se deixar desativado para o uso do script de controle). Os gatilhos RT e LT são respectivamente para avanço e recuo com velocidade determinada pela intensidade do aperto no gatilho. O analógico da esquerda controla as direções de esquerda e direita, compativeis para o lado na qual o analógico for direcionado, com velocidade determinada pelo deslocamento do analógico. O analógico da direita controla avanço, recuo e giro para a esquerda e para direita, respectivo a direção que o analógico apontar, mas com velocidade pré-definida. Os direcionais para cima e para direita acrescentam 5 ao pwm para aumentar a velocidade, e os direcionais para esquerda e para baixo decrescem 5 do pwm para diminuir a velocidade. O pwm começa com 30 e tem seu máximo em 255 e minimo em 30.

* **wally_main_joy_ros_rate:** Pode receber tanto comando do joystick como também comandos de velocidade para cada motor individualmente. E com este código, o arduino fazerá a leitura dos encoders de cada roda e publicará em uma string as quatro leituras com frequência de 20Hz, a atualização e publicação das variáveis é feita por thread. Os comandos do joy são: Botões A, B, X e START, param os motores, Y zera os contadores de pulsos e o botão BACK ativa e desativa comandos de movimento do joy (deve-se deixar desativado para o uso do script de controle). Os gatilhos RT e LT são respectivamente para avanço e recuo com velocidade determinada pela intensidade do aperto no gatilho. O analógico da esquerda controla as direções de esquerda e direita, compativeis para o lado na qual o analógico for direcionado, com velocidade determinada pelo deslocamento do analógico. O analógico da direita controla avanço, recuo e giro para a esquerda e para direita, respectivo a direção que o analógico apontar, mas com velocidade pré-definida. Os direcionais para cima e para direita acrescentam 5 ao pwm para aumentar a velocidade, e os direcionais para esquerda e para baixo decrescem 5 do pwm para diminuir a velocidade. O pwm começa com 30 e tem seu máximo em 255 e minimo em 30.

## Instruções

#### Atenção

Se você vai usar um controle de xbox 360 e não possui o drive instalado ainda, siga este tutorial:
[Configure o controle xbox ubuntu](https://www.edivaldobrito.com.br/configure-o-controle-xbox-ubuntu/)

Para usar este pacote também é necessário ter o pacote [wally_description](https://github.com/GerbersonFelix/wally_description) instalado.

O mapeamento de botôes do controle que está no código pode ser diferente do qual você irá usar. Os scripts que você terá que alterar o mapeamento caso for necessário, será o **due_fake** o do arduino. O mapeamento dos botões pode ser verificado logo no inicio de cada script. Para saber se o que está no script corresponde ao seu controle, execute os seguintes comandos.

```sh
$ rosrun joy joy_node
$ rostopic echo /joy
```
Então faça uma ação (aperte o botão ou desloque no caso do analógico) para verificar se o número que está no script corresponde ao local do vetor mostrado.

##### 1. Instalar todos os pacotes necessários:

```sh
$ sudo apt-get install ros-kinetic-serial
```

##### 2. Faça o download deste pacote para a pasta src do seu catkin workspace:

```sh
$ git clone https://github.com/GerbersonFelix/wally_control_odom.git
$ cd ..
$ catkin_make
```
##### 3. Abrindo o ambiente de simulação:

```sh
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch wally_description wally.launch
```

##### 4. Para controlar o Wally virtual via joystick:

```sh
$ rosrun joy joy_node
$ rosrun wally_control_odom due_fake
$ rosrun wally_control_odom listenner_ticks.py 
$ rosrun wally_control_odom odometry
```

##### 5. Para controlar o Wally real via joystick (Neste caso o controle terá que estar ligado na Jetson do Wally):

```sh
$ rosrun joy joy_node
$ rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
```
