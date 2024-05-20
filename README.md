# TurtleBot Teleoperado

O projeto possui uma interface capaz de detectar em tempo real os botões pressionados pelo usuário e dar um feedback da velocidade do robô em tempo real. Além disso, possui um nó de ROS 2 capaz de comandar o robô utilizando o tópico adequado e que seja capaz de verificar se o robô está inicializado e disponível para receber suas mensagens antes de enviá-las.

![demonstração](gif.gif)

Para ver o vídeo completo de demonstração, [clique aqui](https://drive.google.com/file/d/1fRN1Th9LbQ3ywqTYOBmvrrzRHefTpVnK/view?usp=sharing)

---

### Instalação

Certifique-se de ter o ROS2 instalado em seu sistema. Além disso, certifique-se de ter todas as dependências instaladas, elas estão disponíveis no arquivo `requirements.txt` na raíz do projeto, para isso utilize o comando:

```bash
pip install -r requirements.txt
```

Clone este repositório em seu ambiente:


```bash
git clone https://github.com/rafaelarojas/turtlebot_teleoperado
```

Além disso, clone este repositório também no Robô TurtleBot, para executar o Bringup. Para clonar no robô, utiliza-se o mesmo comando descrito acima.

---

### Execução

#### Execução no robô

1. Para executar no robô, certifique que você está dentro de `workspace/bringup`

2. Abra o terminal na estando dentro da pasta acima e digite:

```bash
ros2 run bringup bringup
```

#### Execução local

1. Para rodar a CLI, certifique que você está na raíz do projeto.

2. Abra outro terminal e rode o seguinte comando (ele só precisa ser executado na primeira vez):

```bash
chmod +x ./exec.sh
```

3. Para executar o código use o comando:

```bash
./exec.sh
```


