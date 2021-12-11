# currito_ros
Proyecto para la asignatura Proyectos de Robótica


Para lanzar joy:

```
rosrun joy joy_node _autorepeat_rate:=10 _coalesce_interval:=0.05
```

Para lanzar rosserial:

```
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
```
## PINES
```
PIN_CEJA_IZQ  2
PIN_CEJA_DER  5
PIN_CRESTA    3
PIN_CUELLO    9
PIN_CUERPO    6
PIN_BOCA      8
1
```
