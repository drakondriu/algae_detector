# algae_detector
-Adicionar os códigos dentro da pasta sarcAndar, rodar :
bash workspace/src/Petrobras_Challenge/start/start.sh 

- Quando o UAV chgar ao primeiro ponto da trajetoria rodar:
python3 img_to_coord.py

- Ele vai coletar tudo o que estiver sendo visto durante o sobrevoo do UAV e vai armazenar suas localizações, é estremamente contraindicado rodar o mapa inteiro dado o numero elevado de pontos, e portanto é sugerido interromper esse comando depois de um certo percurso. depois de interrompido, rodar:
python3 algae_centers
