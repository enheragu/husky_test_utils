# Instrucciones para lanzar los paquetes del Husky

##### 1. En primer lugar ejecutar la función que contiene el archivo `husky_setup.sh` desde el terminal:
  ```sh
    husky_ros_setup 
  ```
  
  Esta función prepara el entorno de ROS. 

##### 2. En segundo lugar ejecutamos:
  ```sh
    husky_launch_sensors
  ```
  
  Esta función lanza todos los nodos de ROS de los sensores y sincroniza el OBC del LIDAR con el del robot.

##### 3. En tercer lugar si se quiera hacer un rosbag existe la función:
  ```sh
    husky_record_rosbag
  ```
  
  Esta función guarda todos los mensaje que se envían a través de los topics que se estén publicando en ese momento.

##### 4. Si se quiere comprobar que el LIDAR está sincronizado:
  ```sh
    rostopic echo /os_cloud_node/points | grep "secs"
    rostopic echo /imu/data | grep "secs" 
  ```
  
  Se observa si coinciden los timestamps de ambos mensajes, por la experincia que tenemos puede haber un delay bastante constante entre amb
os. 

##### 5. Set de la posición de la base del GPS:
  Se precisa del uso de un paquete llamado `test_utils`. Este paqeute contiene una interfaz gráfica a partir de la cual se leen los
 mensajes que el gps publica en el puerto serie.
  ```sh
    python gui_antena_config.py [--port] [--baudrate]
  ```

  En principio no haría falta añadir ningún parámetro, al menos el baudrate está bien configurado. EL puerto puede depende de donde se cone
te el dispositivo.

  En la interfaz se puede hacer un set de unas coordenadas de latitud, longitud y altitud de donde se quiera colocar la base. Para ver si s
e ha quedado guardado, dar al botón read Info y observar que sale. 
