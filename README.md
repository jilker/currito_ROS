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
