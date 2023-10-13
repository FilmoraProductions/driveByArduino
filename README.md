# driveByArduino
controle de borboleta eletronica GM e acelerador eletronico GM com arduino nano e driver ponte h bts-7960, a abertura da valvula funciona a partir da medição de diferença entre o alvo e a posição real (não obtive bons resultados usando o controle PID)
* controle em malha fechada de marcha lenta (usando o sistema PID)
* correção em fase de desaceleração
* mapa de acelerador
* recurso de antilag

 electronic throttle body control with arduino nano and bts-7960 h bridge containing some additional code: (remembering that I am an arduino amateur)
* closed loop idle speed control (PID)
* valve opening control based on the difference between actual position and target position
* deceleration opening correction
* anti-lag feature
* accelerator map
