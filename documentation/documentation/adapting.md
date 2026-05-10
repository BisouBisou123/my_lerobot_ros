# Aanpassen voor custom robot types (welke al een ROS2 interface hebben)

Er kan op eenvoudige wijze een nieuwe robot type worden toegevoegd aan de lerobot-robot-ur package. Hiervoor moet een nieuwe class worden gemaakt die de Ur class erft en de config_class attribute overschrijft met een nieuwe config class die de parameters bevat die nodig zijn voor de nieuwe robot type. De nieuwe config class moet ook de action type bevatten die wordt gebruikt voor de nieuwe robot type.

In het `config_ur.py` bestand is een voorbeeld opgenomen voor een custom robot config. Dit is te vinden in de `class CustomConfig(ROS2Config)`. Zodra deze class is gemaakt, kan de nieuwe robot class worden gemaakt door de Ur class te erven en de config_class attribute te overschrijven met de nieuwe config class. Dit is te vinden in de `class Custom(Ur)`.

In het `ur.py` bestand is een voorbeeld opgenomen voor een custom robot class. Dit is te vinden in de `class Custom(Ur)`. Deze class implementeert dezelfde interface als de Ur class, maar gebruikt een custom config die de action type en andere parameters kan overschrijven.