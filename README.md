# AlicantoFS
Software de control de vuelo de Alicanto
Algunas reglas que seguir para escribir buen codigo de Dronekit en orden de importancia:
* NO USAR vehice.mode en las funciones que llama main.py, perdemos la integridad de el switch de flightmodes en la radio RC, ya que al cambiar de modo de vuelo a algo que no sea GUIDED, dronekit/python salta las lineas que no puede ejecutar (ej. vehicle.simple_goto(wpt)) y pasa a ejecutar las que si puede siendo una de estas vehicle.mode.
* En un mundo ideal no deberiamos usar vehicle.mode, ya que de esta forma al salir de GUIDED usando el switch en la radio, se interrumpe toda ejecucion de dronekit
* Pensar en dronekit como un API y no como un programa que da instrucciones directas al drone. Si el drone encuentra una instruccion de dronekit irrazonable la va a ignorar produciendo comportamientos inesperados.
* Tratar de minimizar el uso de recursos, la Rassberry Pi 3 tiene un procesador de 1.2GHz y 1GB de RAM, no es un computador de escritorio.
* Agregen mas si se les ocurren. 