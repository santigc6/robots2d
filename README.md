# Robot2d

Este proyecto da una implementación basada en Rubenstein _et al._ (1995) de un sistema multiagente en el que un gran grupo de robots son capaces de auto-organizarse formando patrones en 2 dimensiones.

# Instalación

Instalar los paquetes de Python 3 que aparecen en el archivo **requirements.txt**. No obstante, debería ser suficientes con

```{bash}
pip install mesa
pip install matplotlib
pip install jupyter
pip install scipy
```

# Ejecución

Una vez instaladas todas las dependencias, y desde el directorio base (i.e. donde está el archivo **run.py**), se ha de ejecutar el comando
```{bash}
mesa runserver
```

Con ello se abrirá una pestaña en el navegador que nos permitirá pulsar el botón *Start*, o ir paso a paso con el botón *Step*.

# Información adicional

En el directorio **videos** se proporcionan 2 vídeos a velocidad x6 para ver la formación de las dos figuras que contiene el directorio **shapes**. En un futuro, se permitirá elegir la figura que se pretende formar por línea de comandos, y el usuario podrá incluir en el directorio **shapes** figuras propias. Por ahora para elegir entre una figura u otra, se debe editar el fichero **model.py** y cambiar el valor de las variables *MAX_ROBOTS* y *figure1*. Para realizar la figura base
```{python}
MAX_ROBOTS=MAX_ROBOTS1
figure1=figure2
```

Para realizar la figura con un agujero en su interior
```{python}
MAX_ROBOTS=MAX_ROBOTS2
figure1=figure3
```

El resultado que se obtiene en ambas es el que podemos ver en los vídeos comentados anteriormente,

# Referencias

* Rubenstein, M., Cornejo, A., & Nagpal, R. (2014). Programmable self-assembly in a thousand-robot swarm. _Science_, 345(6198), 795-799.
* https://towardsdatascience.com/introduction-to-mesa-agent-based-modeling-in-python-bcb0596e1c9a
* https://github.com/glucee/Multilateration
