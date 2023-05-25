## De que se trata este paquete?
Este paquete está contemplado para ser utilizado en la competencia **MBZIRC 2024**.
Contiene todo lo necesario para poder detectar y localizar botes utilizando una camara montada a un drone.  

Package file structure:  
- **cam_details/** This folder contains camera intrinsics and distortion coefficients data.
- **nodes/** This folder contains all nodes scripts.
- **videos/**  This folder contains all videos for benchmark and testing.
- **weights/** This folder contains the actual weights of trained models.

If you train a new model the **.pt** file should be placed mandatory in **weights/** and only there, the same for any video used as a benchmark but in that case in **videos/**. 


## Stalker node
Nodo principal encargado de la deteccion y localizacion de los botes.

- Subscripciones: 
    - **/drone_posecov**: Pose actual del drone y su respectiva covarianza.
    - **/camera_posecov**: Pose actual de la camara y su respectiva covarianza. 

- Publicaciones: 
    - **/boats_location**:  Coordenadas polares de todos los objetos detectados en la imagen.
    - **/video_stream**: Image frames con los bounding boxes para visualizar la deteccion de los objetos. 

**ULTRA Important**, before starting the node make sure to have define the appropriate params for your system when creating a Stalker node object inside main in **stalker.py**  
```python
Args:
    device (int, optional): Camera to listen for input. 
                            Defaults to 0.
    model (string, optional): Model's file name.
                              Defaults to "yolo_demo.pt"
    classes (list, optional): Which clases to detect while using Yolo model. 
                              Defaults to [32].
    camera_resolution (tuple, optional): Resolution of input camera. 
                                         Defaults to (1920, 1080).
```
Apply your changes here:
```python
def main(args=None):
    rclpy.init(args=args) 
    stalker_node = Stalker(device=0, 
                           classes=[32, 49], 
                           camera_resolution=(640, 640))
    rclpy.spin(stalker_node)
    stalker_node.destroy_node()
    rclpy.shutdown()
```
Now you can initialize the stalker node.
```bash
ros2 run periscope stalker
```

## Video node
Nodo utilizado únicamente para visualizar las detecciones del modelo.
```bash
ros2 run periscope video
```

## Demo node
Nodo utilizado para probar la capacidad de deteccion del modelo entrenado.
Pueden considerarlo como un Benchmark.  

**ULTRA Important**, before starting the node make sure to have define the appropriate model to test and optional the video inside main in **demo.py**  

```python
def main():
    model_file_name = "best_medium.pt"
    video_file_name = "sail_amsterdam.mp4"
```
Now you can initialize the stalker node.
```bash
ros2 run periscope demo
```

 
