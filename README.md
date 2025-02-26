# A-A*pex Multi-Objective Search

Este proyecto implementa el algoritmo A-Apex para optimizar rutas en la red de transporte TransMilenio en Bogotá. El sistema se modela como un grafo dirigido donde las estaciones son nodos y las conexiones tienen costos asociados. Se utilizan criterios multiobjetivo, minimizando el tiempo y la distancia, y maximizando el número de pasajeros transportados, adaptando la heurística para mejorar la eficiencia de búsqueda.

## A-A*pex: Efficient Anytime Approximate Multi-Objective Search

Autores: Han Zhang¹, Oren Salzman², Ariel Felner³, Carlos Hernández Ulloa⁴,⁵,⁶, Sven Koenig¹.

Publicación: [https://ojs.aaai.org/index.php/SOCS/article/view/31556](https://ojs.aaai.org/index.php/SOCS/article/view/31556)

## Integrantes

- [Oscar Iván Ulises Gutiérrez Palacios](mailto:osgutierrezp@unal.edu.co)
- [Juan Diego González Layton](mailto:jgonzalezla@unal.edu.co)
- [Santiago Botero Daza](mailto:sboterod@unal.edu.co)

## Organización del Directorio

- **Carpeta "replicas"**: Código suministrado del repositorio de GitHub de los autores. [Repositorio original](https://github.com/HanZhang39/MultiObjectiveSearch)
- **Carpeta "Análisis"**: Contiene un archivo de Colab con el análisis de error de aproximación de los logs de las ejecuciones de los algoritmos.
- **transmilenio_map y main**: Contiene la implementación de A-Apex para rutas en la red de transporte TransMilenio.

## Deployment

### TransMilenio

Para ejecutar la implementación de A-Apex en la red de TransMilenio:

```bash
python main.py
```

Pruebas de Replicas: Seguir la documentación brindaba por los autores. 

ejemplo de pruebas 
```bash
  ./mohs -m ../maps/NY-m.txt ../maps/NY-t.txt ../maps/NY-d.txt 
--output output_tmp.txt -s 178689 -g 1476 --alg LTMOAR 
--verbal 1 --logsolutions 1
```
